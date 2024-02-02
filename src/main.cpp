/** 0.1 02.01.2024
 *
 * Conner's full rewrite/test firmware for the gra-afch NCS314 Nixie Clock
 * 
**/

/***********************************************************************************************
* * * * * * * * * * * * * * * * * * * * * * INCLUDES * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ClickButton.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <RTClib.h>

/***********************************************************************************************
 * * * * * * * * * * * * * * * * * * * * * * DEFINES * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/

#define TUBES 6
#define US_DateFormat 1
#define EU_DateFormat 0

#define CELSIUS 0
#define FAHRENHEIT 1

#define DS1307_ADDRESS 0x68

#define DOT_MODE_314 0

#define RHV5222PIN 8

// EEPROM data addresses
#define TimeIndex        0
#define DateIndex        1
#define AlarmIndex       2
#define hModeIndex       3
#define TemperatureIndex 4
#define TimeZoneIndex    5
#define TimeHoursIndex   6
#define TimeMintuesIndex 7
#define TimeSecondsIndex 8
#define DateFormatIndex  9
#define DateDayIndex     10
#define DateMonthIndex   11
#define DateYearIndex    12
#define AlarmHourIndex   13
#define AlarmMinuteIndex 14
#define AlarmSecondIndex 15
#define Alarm01          16
#define hModeValueIndex  17
#define DegreesFormatIndex 18
#define HoursOffsetIndex 19

/***********************************************************************************************
 * * * * * * * * * * * * * * * * * * * * * * VARIABLES * * * * * * * * * * * * * * * * * * * * * 
************************************************************************************************/

const unsigned int fpsLimit = 1000; // To set maximum update speed for digit fading

const byte LEpin = 10; // Output enable for both HV5122 HV shift registers

RTC_DS3231 rtc; // RTC object
DateTime dt_now;

bool HV5222;

const int TimeToSleep = 24;
const int TimeToWake = 8;

const byte RedLedPin = 9; // MCU WDM output for red LEDs 9-g
const byte GreenLedPin = 6; // MCU WDM output for green LEDs 6-b
const byte BlueLedPin = 3; // MCU WDM output for blue LEDs 3-r
const byte pinSet = A0;
const byte pinUp = A2;
const byte pinDown = A1;
const byte pinBuzzer = 2;

bool RTC_present = true;

bool TempPresent = false;

const byte RGBLEDsEEPROMAddress = 0;
const byte HourFormatEEPROMAddress = 1;
const byte AlarmTimeEEPROMAddress = 2; //3,4,5
const byte AlarmArmedEEPROMAddress = 6;
const byte LEDsLockEEPROMAddress = 7;
const byte LEDsRedValueEEPROMAddress = 8;
const byte LEDsGreenValueEEPROMAddress = 9;
const byte LEDsBlueValueEEPROMAddress = 10;
const byte DegreesFormatEEPROMAddress = 11;
const byte HoursOffsetEEPROMAddress = 12;
const byte DateFormatEEPROMAddress = 13;
const byte DotsModeEEPROMAddress = 14;

const byte zero = 0x00;

unsigned int NixieArray[10]={1, 2, 4, 8, 16, 32, 64, 128, 256, 512}; // This helps us convert from a single digit decimal (index) to the appropriate binary mask

int colons = 0;
int last_second = 0;

/***********************************************************************************************
 * * * * * * * * * * * * * * * * * * * * * * FUNCTION PROTOTYPES * * * * * * * * * * * * * * * *  
************************************************************************************************/

// Interface setup
void SPISetup();

// Timer setup
void setupTimers();

// Tube operations
void startupTubes();
void updateTubes(const DateTime &now);

// Helper functions
String PreZero(int digit);

// RTC functions
void showDateTime();
//void setRTCDateTime(byte h, byte m, byte s, byte d, byte mon, byte y, byte w = 1);
//void getRTCDateTime();
//void printDateTime();

// Button pins declarations
ClickButton setButton(pinSet, LOW, CLICKBTN_PULLUP);
ClickButton upButton(pinUp, LOW, CLICKBTN_PULLUP);
ClickButton downButton(pinDown, LOW, CLICKBTN_PULLUP);

/***********************************************************************************************
* * * * * * * * * * * * * * * * * * * * Init Program * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/

void setup()
{
  // Stop all interrupts
  //cli();

  // First, init the HV output enable and set it low.
  pinMode(LEpin, OUTPUT);
  digitalWrite(LEpin, LOW);

  Wire.begin();
  rtc.begin();
  Serial.begin(115200);

  SPISetup();

  // Other digital pin inits
  pinMode(pinSet,  INPUT_PULLUP);
  pinMode(pinUp,  INPUT_PULLUP);
  pinMode(pinDown,  INPUT_PULLUP);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);

  // Turn off all LEDs
  analogWrite(RedLedPin,0);
  analogWrite(GreenLedPin,0);
  analogWrite(BlueLedPin,0);

  // Button objects inits
  setButton.debounceTime   = 20;   // Debounce timer in ms
  setButton.multiclickTime = 30;  // Time limit for multi clicks
  setButton.longClickTime  = 2000; // time until "held-down clicks" register

  upButton.debounceTime   = 20;   // Debounce timer in ms
  upButton.multiclickTime = 30;  // Time limit for multi clicks
  upButton.longClickTime  = 2000; // time until "held-down clicks" register

  downButton.debounceTime   = 20;   // Debounce timer in ms
  downButton.multiclickTime = 30;  // Time limit for multi clicks
  downButton.longClickTime  = 2000; // time until "held-down clicks" register

  startupTubes();

  setupTimers();

  last_second = dt_now.second();

  // Turn all LEDs light purple
  analogWrite(RedLedPin,50);
  analogWrite(GreenLedPin,0);
  analogWrite(BlueLedPin,50);

}

void loop()
{
  dt_now = rtc.now();
  if (last_second != dt_now.second())
  {
    colons = !colons;
    updateTubes(dt_now);
    last_second = dt_now.second();
  }
  
  //Serial.print("Hours:");
  //Serial.print(PreZero(now.hour()));
  //_delay_ms(500);
}


/************************************************************************************************
* * * * * * * * * * * * * * * * * * FUNCTION DEFINITIONS ** * * * * * * * * * * * * * * * * * * *
*************************************************************************************************/

/**
 * @brief Initialize SPI after checking if we're using 5122 or 5222 shift reg. 
*/
void SPISetup()
{
  pinMode(RHV5222PIN, INPUT_PULLUP);
  HV5222=!digitalRead(RHV5222PIN);
  SPI.begin(); 
  if (HV5222)
  {
    SPI.beginTransaction(SPISettings(2000000, LSBFIRST, SPI_MODE2));
  } else
  {
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
  }
}

void setupTimers()
{

  const float timerFreq = 1.0; // Timer interval in Hz
  const float timerInterval = 16000000 / timerFreq; // Timer interval
  const int prescaler = 1024;

  //set timer4 interrupt at 1Hz
  TCCR4A = 0; // set entire TCCR1A register to 0
  TCCR4B = 0; // same for TCCR1B
  TCNT4  = 0; //initialize counter value to 0

  // set compare match register for 1hz increments

  //OCR4A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  OCR4A = (int)(timerInterval / prescaler) - 1;

  // turn on CTC mode
  TCCR4B |= (1 << WGM42);

  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS42) | (1 << CS40); 
  //TCCR1B |= prescaler;  // Set the prescaler

  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  // Enable global interrupts
  sei();

}

/**
 * @brief Function to test some routines with the high voltage shift register. 
*/
void startupTubes()
{
  int delay = 25; // this must never be less than 1ms (I think)

  digitalWrite(LEpin, LOW);

  uint32_t nixie_tube_data = 0;
  SPI.transfer(nixie_tube_data>>24);
  SPI.transfer(nixie_tube_data>>16);
  SPI.transfer(nixie_tube_data>>8);
  SPI.transfer(nixie_tube_data);

  nixie_tube_data = 0;
  SPI.transfer(nixie_tube_data>>24);
  SPI.transfer(nixie_tube_data>>16);
  SPI.transfer(nixie_tube_data>>8);
  SPI.transfer(nixie_tube_data);

  digitalWrite(LEpin, LOW);

  _delay_ms(delay);

  nixie_tube_data = 0x0000000F;

  for (int i = 0; i < 30; i++)
  {
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    SPI.transfer(nixie_tube_data>>24);
    SPI.transfer(nixie_tube_data>>16);
    SPI.transfer(nixie_tube_data>>8);
    SPI.transfer(nixie_tube_data);

    nixie_tube_data = nixie_tube_data<<1;

    digitalWrite(LEpin, HIGH);
    _delay_ms(delay);
    digitalWrite(LEpin, LOW);
  }

  nixie_tube_data = 0x0000000F;
  
  for (int i = 0; i < 30; i++)
  {
    SPI.transfer(nixie_tube_data>>24);
    SPI.transfer(nixie_tube_data>>16);
    SPI.transfer(nixie_tube_data>>8);
    SPI.transfer(nixie_tube_data);

    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    nixie_tube_data = nixie_tube_data<<1;

    digitalWrite(LEpin, HIGH);
    _delay_ms(delay);
    digitalWrite(LEpin, LOW);
  }

}

String PreZero(int digit)
{
  digit = abs(digit);
  if (digit < 10) return String("0") + String(digit);
  else return String(digit);
}

void showDateTime()
{
  DateTime dt = rtc.now();

  Serial.print((dt.year()), DEC);
  Serial.print('/');
  Serial.print(PreZero(dt.month()));
  Serial.print('/');
  Serial.print(PreZero(dt.day()));
  Serial.print(' ');
  Serial.print(PreZero(dt.hour()));
  Serial.print(':');
  Serial.print(PreZero(dt.minute()));
  Serial.print(':');
  Serial.print(PreZero(dt.second()));

  Serial.println();
}

ISR(TIMER4_COMPA_vect)
{
  //cli();
  //DateTime now = rtc.now();
  //updateTubes(now);
  //showDateTime();
  //colons = !colons;
  //dt_now = rtc.now();
  //Serial.print("Colons: ");
  //Serial.println(colons);
  //sei();
  
}

void updateTubes(const DateTime &now)
{
  digitalWrite(LEpin, LOW);

  unsigned long chhm = 0; // data structure for colon 1, hour ones, hour tens, and minute tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // update blinking colons mask
  chhm |= (unsigned long) colons << 31 | (unsigned long)colons << 30;
  cssm |= (unsigned long) colons << 31 | (unsigned long)colons << 30;

  // update with current time data
  // int ht = now.hour() / 10;
  // int ho = now.hour() % 10;
  // int mt = now.minute() / 10;
  // int mo = now.minute() % 10;
  // int st = now.second() / 10;
  // int so = now.second() % 10;
  //chhm |= (unsigned long) NixieArray[now.hour() % 10] << 30 | (unsigned long) NixieArray[now.hour() / 10] << 20 | (unsigned long) NixieArray[now.minute() / 10] << 10;
  //cssm |= (unsigned long) NixieArray[now.second() % 10] << 30 | (unsigned long) NixieArray[now.second() / 10] << 20 | (unsigned long) NixieArray[now.minute() % 10] << 10;
  
  // Convert from 24hr to 12hr time
  int hour = 0;
  if (now.hour() == 0)
  { 
    hour = 12;
  } 
  else if (now.hour() > 12)
  {
    hour = now.hour() - 12;
  }
  else 
  {
    hour = now.hour();
  }

  chhm |= (unsigned long) NixieArray[now.minute() / 10] << 20 | (unsigned long) NixieArray[hour % 10] << 10 | (unsigned long) NixieArray[hour / 10];
  cssm |= (unsigned long) NixieArray[now.second() % 10] << 20 | (unsigned long) NixieArray[now.second() / 10] << 10 | (unsigned long) NixieArray[now.minute() % 10];

  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(chhm>>24);
  SPI.transfer(chhm>>16);
  SPI.transfer(chhm>>8);
  SPI.transfer(chhm);

  digitalWrite(LEpin, HIGH);

}