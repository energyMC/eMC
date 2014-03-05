/*  Energy monitor and solar power diverter for solar PV system
    based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    this version implements a phase-locked loop to synchronise to the 50Hz supply and
    supports a single Dallas DS18B20 temperature sensor.
    
    The triac driver should be connected to the pulse jack via a suitable series resistor.

    Author: Martin Roberts 2/12/12
    
    History:
    2/12/12  first published version
    3/12/12  diverted power calculation & transmission added
    4/12/12  manual power input added for testing
    10/12/12 high & low energy thresholds added to reduce flicker
    12/10/13 PB added 3rd CT channel to determine diverted power                    // PB added line
*/

//--------------------------------------------------------------------------------------------------
// constants which must be set for each system
#define VCAL 233.5  // calculated value is 230:9 for transformer x 11:1 for resistor divider = 281
#define I1CAL 112.6 // calculated value is 100A:0.05A for transformer / 18 Ohms for resistor = 111.1
#define I2CAL 94.8 // this is for CT2, the solar PV current transformer
#define I3CAL 94.8                                                                         //PB added this line
#define I1LEAD 5 // number of microseconds the CT1 input leads the voltage input by
#define I2LEAD 5 // number of microseconds the CT2 input leads the voltage input by
#define I3LEAD 5                                                                           //PB added this line
#define POWERCORRECTION 0 // this value, in watts, may be used to compensate for the leakage from
                          //  voltage to current inputs, it only affects data sent to emonGLCD
#define LOAD_POWER 2770 // power in watts (at 240V) of triac load for diverted power calculation
//#define LEDISLOCK // comment this out for LED pulsed during transmission
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// other system constants
#define SUPPLY_VOLTS 3.3 // used here because it's more accurate than the internal band-gap reference
#define SUPPLY_FREQUENCY 50
#define NUMSAMPLES 50 // number of times to sample each 50Hz cycle
#define ENERGY_BUFFER_SIZE 3600 // 0.001 kWh = 3600 Joules
#define BUFFER_HIGH_THRESHOLD 2700 // energy buffer level to start diversion
#define BUFFER_LOW_THRESHOLD 900 // energy buffer level to stop diversion
#define FILTERSHIFT 13 // for low pass filters to determine ADC offsets
#define PLLTIMERRANGE 100 // PLL timer range limit ~+/-0.5Hz
#define PLLLOCKRANGE 40 // allowable ADC range to enter locked state
#define PLLUNLOCKRANGE 80 // allowable ADC range to remain locked
#define PLLLOCKCOUNT 100 // number of cycles to determine if PLL is locked
#define LOOPTIME 5000 // time of outer loop in milliseconds, also time between radio transmissions
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// constants calculated at compile time
#define V_RATIO ((VCAL * SUPPLY_VOLTS)/1024)
#define I1_RATIO ((I1CAL * SUPPLY_VOLTS)/1024)
#define I2_RATIO ((I2CAL * SUPPLY_VOLTS)/1024)
#define I3_RATIO ((I3CAL * SUPPLY_VOLTS)/1024)                                             // PB added line
#define I1PHASESHIFT (((I1LEAD+63)*256)/400) // phase shift in voltage to align to current samples
#define I2PHASESHIFT (((I2LEAD+127)*256)/400) //  these are fractions (x256) of sample period
#define I3PHASESHIFT (((I3LEAD+191)*256)/400)                                              // PB added line
#define JOULES_PER_BUFFER_UNIT ((V_RATIO * I1_RATIO)/(SUPPLY_FREQUENCY*NUMSAMPLES))
#define MAXAVAILABLEENERGY ((long)ENERGY_BUFFER_SIZE/JOULES_PER_BUFFER_UNIT)
#define HIGHENERGYLEVEL ((long)BUFFER_HIGH_THRESHOLD/JOULES_PER_BUFFER_UNIT)
#define LOWENERGYLEVEL ((long)BUFFER_LOW_THRESHOLD/JOULES_PER_BUFFER_UNIT)
#define FILTERROUNDING (1<<(FILTERSHIFT-1))
#define TIMERTOP (((20000/NUMSAMPLES)*16)-1) // terminal count for PLL timer
#define PLLTIMERMAX (TIMERTOP+PLLTIMERRANGE)
#define PLLTIMERMIN (TIMERTOP-PLLTIMERRANGE)
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Arduino I/O pin useage
#define VOLTSPIN 2
#define CT1PIN 3
#define CT2PIN 0
#define CT3PIN 1                                                                          // PB added line
#define LEDPIN 9
#define SYNCPIN 6 // this output will be a 50Hz square wave locked to the 50Hz input
#define SAMPPIN 5 // this output goes high each time an ADC conversion starts or completes
#define RFMSELPIN 10
#define RFMIRQPIN 2
#define SDOPIN 12
#define W1PIN 4 // 1-Wire pin for temperature
#define TRIACPIN 3 // triac driver pin
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------
// Dallas DS18B20 commands
#define SKIP_ROM 0xcc 
#define CONVERT_TEMPERATURE 0x44
#define READ_SCRATCHPAD 0xbe
#define BAD_TEMPERATURE 30000 // this value (300C) is sent if no sensor is present
//--------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>

typedef struct { int power1, power2, power3, power4, Vrms, temp; } PayloadTx;             // PB added , power4
PayloadTx emontx;

int sampleV,sampleI1,sampleI2,sampleI3,numSamples;                                        // PB added samplesI3,
int voltsOffset=512,I1Offset=512,I2Offset=512,I3Offset=512; //start offsets at ADC centre // PB added ,I3Offset=512
float Vrms,I1rms,I2rms,I3rms;                                                             // PB added ,I3rms
long sumVsquared,sumI1squared,sumI2squared,sumI3squared,sumP1,sumP2,sumP3;                // PB added sumI3squared and sumP3
long cycleVsquared,cycleI1squared,cycleI2squared,cycleI3squared,cycleP1,cycleP2,cycleP3;  // PB added cycleI3squared and cycleP3
long totalVsquared,totalI1squared,totalI2squared,totalI3squared,totalP1,totalP2,totalP3;  // PB added totalI3squared and totalP3
long sumTimerCount;
float realPower1,apparentPower1,powerFactor1;
float realPower2,apparentPower2,powerFactor2;
float realPower3,apparentPower3,powerFactor3;    // PB added line
float divertedPower;
float frequency;
word timerCount=TIMERTOP;
word pllUnlocked=PLLLOCKCOUNT;
word cycleCount;
boolean newCycle,divertedCycle;
int divertedCycleCount;
unsigned long nextTransmitTime;
long availableEnergy;
int manualPowerLevel;

OneWire oneWire(W1PIN);

void setup()
{
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  pinMode(SYNCPIN, OUTPUT);
  digitalWrite(SYNCPIN, LOW);
  pinMode(SAMPPIN, OUTPUT);
  digitalWrite(SAMPPIN, LOW);
  pinMode(TRIACPIN,OUTPUT);
  digitalWrite(TRIACPIN,LOW);
  pinMode (RFMSELPIN, OUTPUT);
  digitalWrite(RFMSELPIN,HIGH);
  manualPowerLevel=0;
  convertTemperature(); // start initial temperature conversion
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  // initialise RFM12
  delay(200); // wait for RFM12 POR
  rfm_write(0x0000); // clear SPI
  rfm_write(0x80D7); // EL (ena dreg), EF (ena RX FIFO), 12.0pF 
  rfm_write(0x8208); // Turn on crystal,!PA
  rfm_write(0xA640); // 434MHz 
  rfm_write(0xC606); // approx 49.2 Kbps, as used by emonTx
  //rfm_write(0xC657); // approx 3.918 Kbps, better for long range
  rfm_write(0xCC77); // PLL 
  rfm_write(0x94A0); // VDI,FAST,134kHz,0dBm,-103dBm 
  rfm_write(0xC2AC); // AL,!ml,DIG,DQD4 
  rfm_write(0xCA83); // FIFO8,2-SYNC,!ff,DR 
  rfm_write(0xCEd2); // SYNC=2DD2
  rfm_write(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
  rfm_write(0x9850); // !mp,90kHz,MAX OUT 
  rfm_write(0xE000); // wake up timer - not used 
  rfm_write(0xC800); // low duty cycle - not used 
  rfm_write(0xC000); // 1.0MHz,2.2V 
  
  nextTransmitTime=millis();
  Serial.begin(9600);
  
  // change ADC prescaler to /64 = 250kHz clock
  // slightly out of spec of 200kHz but should be OK
  ADCSRA &= 0xf8;  // remove bits set by Arduino library
  ADCSRA |= 0x06; 

  //set timer 1 interrupt for required period
  noInterrupts();
  TCCR1A = 0; // clear control registers
  TCCR1B = 0;
  TCNT1  = 0; // clear counter
  OCR1A = TIMERTOP; // set compare reg for timer period
  bitSet(TCCR1B,WGM12); // CTC mode
  bitSet(TCCR1B,CS10); // no prescaling
  bitSet(TIMSK1,OCIE1A); // enable timer 1 compare interrupt
  bitSet(ADCSRA,ADIE); // enable ADC interrupt
  interrupts();
}

void loop()
{
  if(newCycle) addCycle(); // a new mains cycle has been sampled

  if((millis()>=nextTransmitTime) && ((millis()-nextTransmitTime)<0x80000000L)) // check for overflow
  {
#ifndef LEDISLOCK
    digitalWrite(LEDPIN,HIGH);
#endif
    calculateVIPF();
    emontx.temp=readTemperature();
    sendResults();
    convertTemperature(); // start next conversion
    nextTransmitTime+=LOOPTIME;
#ifndef LEDISLOCK
    digitalWrite(LEDPIN,LOW);
#endif
  }

  if(Serial.available())
  {
    manualPowerLevel=Serial.parseInt();
    manualPowerLevel=constrain(manualPowerLevel,0,255);
    Serial.print("manual power level set to ");
    Serial.println(manualPowerLevel);
  }
}

// timer 1 interrupt handler
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(SAMPPIN,HIGH);
  ADMUX = _BV(REFS0) | VOLTSPIN; // start ADC conversion for voltage
  ADCSRA |= _BV(ADSC);
  digitalWrite(SAMPPIN,LOW);
}

// ADC interrupt handler
ISR(ADC_vect)
{
  static int newV,newI1,newI2,newI3;    // PB added newI3
  static int lastV;
  static long fVoltsOffset=512L<<FILTERSHIFT,fI1Offset=512L<<FILTERSHIFT,fI2Offset=512L<<FILTERSHIFT,fI3Offset=512L<<FILTERSHIFT;    // PB added fI3Offset=512L<<FILTERSHIFT
  int result;
  long phaseShiftedV;
  
  digitalWrite(SAMPPIN,HIGH);
  result = ADCL;
  result |= ADCH<<8;

  // determine which conversion just completed
  switch(ADMUX & 0x0f)
  {
    case VOLTSPIN:
      ADMUX = _BV(REFS0) | CT1PIN; // start CT1 conversion
      ADCSRA |= _BV(ADSC);
      lastV=newV;
      sampleV = result;
      newV=sampleV-voltsOffset;
      sumVsquared+=((long)newV*newV);
      // update low-pass filter for DC offset
      fVoltsOffset += (sampleV-voltsOffset); 
      voltsOffset=(int)((fVoltsOffset+FILTERROUNDING)>>FILTERSHIFT);
      // determine voltage at current sampling points and use it for power calculation
      phaseShiftedV=lastV+((((long)newV-lastV)*I1PHASESHIFT)>>8);
      sumP1+=(phaseShiftedV*newI1);
      phaseShiftedV=lastV+((((long)newV-lastV)*I2PHASESHIFT)>>8);
      sumP2+=(phaseShiftedV*newI2);
      phaseShiftedV=lastV+((((long)newV-lastV)*I3PHASESHIFT)>>8);   // PB added line
      sumP3+=(phaseShiftedV*newI3);                                 // PB added line
      break;
    case CT1PIN:
      ADMUX = _BV(REFS0) | CT2PIN; // start CT2 conversion
      ADCSRA |= _BV(ADSC);
      sampleI1 = result;
      newI1=sampleI1-I1Offset;
      sumI1squared+=((long)newI1*newI1);
      fI1Offset += (sampleI1-I1Offset); 
      I1Offset=(int)((fI1Offset+FILTERROUNDING)>>FILTERSHIFT);
      break;
    case CT2PIN:
      ADMUX = _BV(REFS0) | CT3PIN; // start CT3 conversion          // PB added line
      ADCSRA |= _BV(ADSC);                                          // PB added line
      sampleI2 = result;
      newI2=sampleI2-I2Offset;
      sumI2squared+=((long)newI2*newI2);
      fI2Offset += (sampleI2-I2Offset);
      I2Offset=(int)((fI2Offset+FILTERROUNDING)>>FILTERSHIFT);
      break;                                                        // PB added line
    case CT3PIN:                                                    // PB added line
      sampleI3 = result;                                            // PB added line
      newI3=sampleI3-I3Offset;                                      // PB added line
      sumI3squared+=((long)newI3*newI3);                            // PB added line
      fI3Offset += (sampleI3-I3Offset);                             // PB added line
      I3Offset=(int)((fI3Offset+FILTERROUNDING)>>FILTERSHIFT);      // PB added line
      updatePLL(newV,lastV);
      break;
  }
  digitalWrite(SAMPPIN,LOW);
}

void updatePLL(int newV, int lastV)
{
  static byte samples=0;
  static int oldV;
  static boolean divertFlag, diverting=false;
  static int manualCycleCount=-1;
  boolean rising;

  rising=(newV>lastV); // synchronise to rising zero crossing
  
  samples++;
  if(samples>=NUMSAMPLES) // end of one 50Hz cycle
  {
    digitalWrite(SYNCPIN,HIGH);
    samples=0;
    if(rising)
    {
      // if we're in the rising part of the 50Hz cycle adjust the final timer count
      // to move newV towards 0, only adjust if we're moving in the wrong direction
      if(((newV<0)&&(newV<=oldV))||((newV>0)&&(newV>=oldV))) timerCount-=newV;
      // limit range of PLL frequency
      timerCount=constrain(timerCount,PLLTIMERMIN,PLLTIMERMAX);
      OCR1A=timerCount;
      if(abs(newV)>PLLUNLOCKRANGE) pllUnlocked=PLLLOCKCOUNT; // we're unlocked
      else if(pllUnlocked) pllUnlocked--;
#ifdef LEDISLOCK
      digitalWrite(LEDPIN,pllUnlocked?LOW:HIGH);
#endif
    }
    else // in the falling part of the cycle, we shouldn't be here
    {
      OCR1A=PLLTIMERMAX; // shift out of this region fast
      pllUnlocked=PLLLOCKCOUNT; // and we can't be locked
    }
    
    oldV=newV;
    
    // save results for outer loop
    cycleVsquared=sumVsquared;
    cycleI1squared=sumI1squared;
    cycleI2squared=sumI2squared;
    cycleI3squared=sumI3squared;   // PB added line
    cycleP1=sumP1;
    cycleP2=sumP2;
    cycleP3=sumP3;                 // PB added line
    divertedCycle=divertFlag;
    // and clear accumulators
    sumVsquared=0;
    sumI1squared=0;
    sumI2squared=0;
    sumI3squared=0;                // PB added line
    sumP1=0;
    sumP2=0;
    sumP3=0;                       // PB added line
    divertFlag=false;
    newCycle=true; // flag new cycle to outer loop
    if(manualPowerLevel) manualCycleCount++;
    else manualCycleCount=-1; // manual power is off
  }
  else if(samples==(NUMSAMPLES/2))
  {
    // negative zero crossing
    digitalWrite(SYNCPIN,LOW);
  }
  else if(samples==((NUMSAMPLES/2)-4)) // fire triac ~1.6ms before -ve zero crossing
  {
    if(availableEnergy > HIGHENERGYLEVEL) diverting=true;
    else if(availableEnergy < LOWENERGYLEVEL) diverting=false;
    
    if(diverting || (manualCycleCount>=manualPowerLevel))
    {
      digitalWrite(TRIACPIN,HIGH);
      divertFlag=true;
      manualCycleCount=0;
    }
    else digitalWrite(TRIACPIN,LOW);
  }
  digitalWrite(SAMPPIN,LOW);
}

// add data for new 50Hz cycle to total
void addCycle()
{
  totalVsquared+=cycleVsquared;
  totalI1squared+=cycleI1squared;
  totalI2squared+=cycleI2squared;
  totalI3squared+=cycleI3squared;    // PB added line
  totalP1+=cycleP1;
  totalP2+=cycleP2;
  totalP3+=cycleP3;                  // PB added line
  numSamples+=NUMSAMPLES;
  sumTimerCount+=(timerCount+1); // for average frequency calculation
  availableEnergy-=cycleP1; // Solar energy is negative at this point
  availableEnergy=constrain(availableEnergy,0,MAXAVAILABLEENERGY);
  if(divertedCycle) divertedCycleCount++;
  cycleCount++;
  newCycle=false;
}

// calculate voltage, current, power and frequency
void calculateVIPF()
{
  if(numSamples==0) return; // just in case
  
  Vrms = V_RATIO * sqrt(((float)totalVsquared)/numSamples); 
  I1rms = I1_RATIO * sqrt(((float)totalI1squared)/numSamples); 
  I2rms = I2_RATIO * sqrt(((float)totalI2squared)/numSamples); 
  I3rms = I3_RATIO * sqrt(((float)totalI3squared)/numSamples);      // PB added line
  realPower1 = (V_RATIO * I1_RATIO * (float)totalP1)/numSamples;
  if(abs(realPower1)>POWERCORRECTION) realPower1-=POWERCORRECTION;
  apparentPower1 = Vrms * I1rms;
  powerFactor1=abs(realPower1 / apparentPower1);
  realPower2 = (V_RATIO * I2_RATIO * (float)totalP2)/numSamples;
  if(abs(realPower2)>POWERCORRECTION) realPower2-=POWERCORRECTION;
  apparentPower2 = Vrms * I2rms;
  powerFactor2=abs(realPower2 / apparentPower2);
  realPower3 = (V_RATIO * I3_RATIO * (float)totalP3)/numSamples;    // PB added line
  if(abs(realPower3)>POWERCORRECTION) realPower3-=POWERCORRECTION;  // PB added line
  apparentPower3 = Vrms * I3rms;                                    // PB added line
  powerFactor3=abs(realPower3 / apparentPower3);                    // PB added line
  divertedPower=((float)divertedCycleCount*LOAD_POWER)/cycleCount;
  divertedPower=divertedPower*(Vrms/240)*(Vrms/240); // correct power for actual voltage
  frequency=((float)cycleCount*16000000)/(((float)sumTimerCount)*NUMSAMPLES);

  emontx.power1=(int)(realPower1+0.5);
  emontx.power2=(int)(realPower2+0.5);
  emontx.power3=(int)(realPower3+0.5);        // PB added line
  emontx.power4=(int)(divertedPower+0.5);     // PB changed power3 to power4

  emontx.Vrms=(int)(Vrms*100+0.5);
  
  totalVsquared=0;
  totalI1squared=0;
  totalI2squared=0;
  totalI3squared=0;          // PB added line
  totalP1=0;
  totalP2=0;
  totalP3=0;                 // PB added line
  numSamples=0;
  cycleCount=0;
  divertedCycleCount=0;
  sumTimerCount=0;
}

void sendResults()
{
  rfm_send((byte *)&emontx,sizeof(emontx));
  Serial.print(voltsOffset);
  Serial.print(" ");
  Serial.print(I1Offset);
  Serial.print(" ");
  Serial.print(I2Offset);
  Serial.print(" ");
  Serial.print(I3Offset);    // PB added line
  Serial.print(" ");         // PB added line
  Serial.print(Vrms);
  Serial.print(" ");
  Serial.print(realPower1);
  Serial.print(" ");
  Serial.print(realPower2);
  Serial.print(" ");
  Serial.print(realPower3);  // PB added line
  Serial.print(" ");         // PB added line
  Serial.print(divertedPower);
  Serial.print(" ");
  Serial.print(frequency);
  Serial.print(" ");
  Serial.print((float)emontx.temp/100);
  Serial.print(" ");
  Serial.print((float)availableEnergy * JOULES_PER_BUFFER_UNIT);
  if(pllUnlocked) Serial.print(" PLL is unlocked ");
  else Serial.print(" PLL is locked ");
  Serial.println();
}

// write a command to the RFM12
word rfm_write(word cmd)
{
  word result;
  
  digitalWrite(RFMSELPIN,LOW);
  result=(SPI.transfer(cmd>>8)<<8) | SPI.transfer(cmd & 0xff);
  digitalWrite(RFMSELPIN,HIGH);
  return result;
}

// transmit data via the RFM12
void rfm_send(byte *data, byte size)
{
  byte i=0,next,txstate=0;
  word crc=~0;
  
  rfm_write(0x8228); // OPEN PA
  rfm_write(0x8238);

  digitalWrite(RFMSELPIN,LOW);
  SPI.transfer(0xb8); // tx register write command
  
  while(txstate<13)
  {
    while(digitalRead(SDOPIN)==0); // wait for SDO to go high
    switch(txstate)
    {
      case 0:
      case 1:
      case 2: next=0xaa; txstate++; break;
      case 3: next=0x2d; txstate++; break;
      case 4: next=0xd2; txstate++; break;
      case 5: next=10; txstate++; break; // node ID
      case 6: next=size; txstate++; break;
      case 7: next=data[i++]; if(i==size) txstate++; break;
      case 8: next=(byte)crc; txstate++; break;
      case 9: next=(byte)(crc>>8); txstate++; break;
      case 10:
      case 11:
      case 12: next=0xaa; txstate++; break; // dummy bytes (if <3 CRC gets corrupted sometimes)
    }
    if((txstate>4)&&(txstate<9)) crc = _crc16_update(crc, next);
    SPI.transfer(next);
  }
  digitalWrite(RFMSELPIN,HIGH);

  rfm_write( 0x8208 ); // CLOSE PA
  rfm_write( 0x8200 ); // enter sleep
}

void convertTemperature()
{
  oneWire.reset();
  oneWire.write(SKIP_ROM);
  oneWire.write(CONVERT_TEMPERATURE);
}

int readTemperature()
{
  byte buf[9];
  int result;
  
  oneWire.reset();
  oneWire.write(SKIP_ROM);
  oneWire.write(READ_SCRATCHPAD);
  for(int i=0; i<9; i++) buf[i]=oneWire.read();
  if(oneWire.crc8(buf,8)==buf[8])
  {
    result=(buf[1]<<8)|buf[0];
    // result is temperature x16, multiply by 6.25 to convert to temperature x100
    result=(result*6)+(result>>2);
  }
  else result=BAD_TEMPERATURE;
  return result;
}

