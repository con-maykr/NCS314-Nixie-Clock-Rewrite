/** 0.1 02.01.2024
 *
 * Conner's full rewrite/test firmware for the gra-afch NCS314 Nixie Clock
 * 
**/

/***********************************************************************************************
* * * * * * * * * * * * * * * * * * * * * * INCLUDES * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/
#define __DELAY_BACKWARD_COMPATIBLE__
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ClickButton.h>
#include <TimeLib.h>
#include <EEPROM.h>
//#include <OneWire.h>
#include <RTClib.h>
#include <string.h>

/***********************************************************************************************
 * * * * * * * * * * * * * * * * * * * * * * DEFINES * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/

#define TUBES 6
#define US_DateFormat 1
#define EU_DateFormat 0

#define CELSIUS 0
#define FAHRENHEIT 1

#define DS1307_ADDRESS 0x68

/** 
 * Dot Modes:
 * 0: Always off
 * 1: Always on
 * 2: Blink every other second
*/
#define DOT_MODE 1

// Smoothly fade out of each digit
#define FADING 1

#define RHV5222PIN 8

// Truths for how we arrange hours/minutes/seconds in a digit array. T = tens, O = ones
#define HOUR_T 0
#define HOUR_O 1
#define MIN_T 2
#define MIN_O 3
#define SEC_T 4
#define SEC_O 5

// Shortcut for when we don't need to use the mask
// Actually this should just be implemented as functions with different parameter sets
#define ALL_ON = {1, 1, 1, 1, 1, 1}

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

const bool RTC_present = true;

const bool TempPresent = false;

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

const unsigned int NixieArray[10]={1, 2, 4, 8, 16, 32, 64, 128, 256, 512}; // This helps us convert from a single digit decimal (index) to the appropriate binary mask

// GLOBAL VARIABLES

RTC_DS3231 rtc; // Real time clock object
// DateTime dt_last; // DateTime object to hold the last one during loops.
bool firstRun = true;

/***********************************************************************************************
 * * * * * * * * * * * * * * * * * * * * * * FUNCTION PROTOTYPES * * * * * * * * * * * * * * * *  
************************************************************************************************/

// Interface setup
void SPISetup();

// Timer setup
void setupTimers();

// Tube operations
void startupTubes();
void updateNixieTime(const DateTime &last, const DateTime &now);
void antiBurn(const DateTime &now);
void fadeTubes(const int *mask, const DateTime &dt);

void sendTime(const int *mask, const DateTime &now);
void sendTime(const DateTime &now);

void sendDigits(const int *mask, int dots, const int *digits);
void sendDigits(int dots, const int *digits);

void sendString(int dots, const char *str);

// Helper functions
String PreZero(int digit);
int hours24to12(int hours24);

// RTC functions

// Eeprom operations
void dumpEeprom();

// Button pins declarations
ClickButton setButton(pinSet, LOW, CLICKBTN_PULLUP);
ClickButton upButton(pinUp, LOW, CLICKBTN_PULLUP);
ClickButton downButton(pinDown, LOW, CLICKBTN_PULLUP);

/***********************************************************************************************
* * * * * * * * * * * * * * * * * * * * Init Program * * * * * * * * * * * * * * * * * * * * * *
************************************************************************************************/

/**
 * @brief Setup function, runs once before program loops. 
*/
void setup()
{
  // Stop all interrupts
  //cli();
  
  Serial.begin(115200);
  
  // First, init the HV output enable and set it low.
  pinMode(LEpin, OUTPUT);
  digitalWrite(LEpin, LOW);

  Wire.begin();

  if (! rtc.begin()){
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  SPISetup();
  // Serial.println("SPI initialized.");

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

  // Quick wipe-across test of all nixie elements
  //startupTubes();

  // Start up the hardware timer(s) to be used for interrupts
  // setupTimers();

  // grab the current second to start comparing in the loop.
  // dt_last = rtc.now();

  // Test the anti burn-in on start
  antiBurn(rtc.now());

  // Dump the eeprom data to serial on program load
  // dumpEeprom();
  
  // Serial.println("Setup done");
  // Serial.println("Current RTC time:");
  // Serial.print(rtc.now().hour());
  // Serial.print(":");
  // Serial.print(rtc.now().minute());
  // Serial.print(":");
  // Serial.println(rtc.now().second());

  int mask[6] = {1,1,0,0,1,1};
  int digits[6] = {6,9,0,0,6,9};
  sendDigits(mask, 0, digits);
  digitalWrite(LEpin, HIGH);
  _delay_ms(1000);
  digitalWrite(LEpin, LOW);

}

/**
 * @brief Main program loop. 
*/
void loop()
{
  
  static DateTime dt_last;
  static bool antiBurn_ON;

  antiBurn_ON = (rtc.now().hour() < 9) || (rtc.now().hour() > 18);

  if (firstRun)
  {
    dt_last = rtc.now();
    firstRun = false;
  }

  DateTime dt_now = rtc.now();// Hold the current time that we update

  if (dt_now.second() != dt_last.second())
  {
    updateNixieTime(dt_last, dt_now);
    dt_last = rtc.now();

  }

  // "Slot machine" routine to prevent cathode poisoning
  // if ( (dt_now.second() % 30 == 0) && ( (dt_now.hour() < 7) || (dt_now.hour() > 22) )) // do it every 30 sec between 10pm and 7am
  if ((dt_now.second() % 60 == 0) && antiBurn_ON) // do it every 30 sec between 10pm and 7am
  {
    antiBurn(dt_now);
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

/**
 * @brief Init hardware timer and enable interrupts. 
*/
void setupTimers()
{

  const float timerFreq = 1.0; // Timer interval in Hz
  const float timerInterval = 16000000 / timerFreq; // Timer interval
  const int prescaler = 1024;

  //set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0

  // set compare match register for 1hz increments

  //OCR1A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  OCR1A = (int)(timerInterval / prescaler) - 1;

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);

  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10); 
  //TCCR1B |= prescaler;  // Set the prescaler

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Enable global interrupts
  sei();

}

/**
 * @brief Apphend a leading zero to an int if it is one digit. 
*/
String PreZero(int digit)
{
  digit = abs(digit);
  if (digit < 10) return String("0") + String(digit);
  else return String(digit);
}

/**
 * @brief Convert 24 hour time format to 12 hour format (loses AM/PM data)
*/
int hours24to12(int hours24)
{
  return hours24 == 0 ? 12 : hours24 <= 12 ? hours24 : hours24 - 12;
}

/**
 * @brief ISR triggered by hardware timer 4 overload. 
*/
ISR(TIMER1_COMPA_vect)
{
  //cli();
  //DateTime now = rtc.now();
  //updateNixieTime(now);
  //showDateTime();
  //colons = !colons;
  //dt_now = rtc.now();
  //Serial.print("Colons: ");
  //Serial.println(colons);
  //sei();
  
}

/**
 * @brief Quickly test all tube elements. 
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

/**
 * @brief Function that updates the state of the nixie tubes against the passed-in DateTime object. 
*/
void updateNixieTime(const DateTime &last, const DateTime &now)
{
  digitalWrite(LEpin, LOW);
  
  // create a mask {H, h, M, m, S, s} where 1 indicates a digit is unchanging from last time to current time
  int mask[6] = { 
    (last.hour() / 10 == now.hour() / 10),
    (last.hour() % 10 == now.hour() % 10),
    (last.minute() / 10 == now.minute() / 10),
    (last.minute() % 10 == now.minute() % 10),
    (last.second() / 10 == now.second() / 10),
    (last.second() % 10 == now.second() % 10) 
    };

  if (FADING)
  {
    fadeTubes(mask, last);
  }
  

  sendTime(now);

  //digitalWrite(LEpin, HIGH);
  analogWrite(LEpin, 200); // Keeps the brightness consistent with the PWM inherent to fade-out multiplexing

}

/**
 * @brief Cycles all tube elements simultaneously to prevent burn-in 
*/
void antiBurn(const DateTime &now)
{

  digitalWrite(LEpin, LOW);

  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // Fill an int array with the current time (easier to loop over an array)
  int time[6] = { 
    now.hour() / 10, 
    now.hour() % 10,
    now.minute() / 10,
    now.minute() % 10,
    now.second() / 10,
    now.second() % 10 
  };
                
  for (int num = 0; num < 10; num++)
  {
    for (int digit = 0; digit < 6; digit++) // [ |5| |4| : |3| |2| : |1| |0| ]
    {
      if (time[digit] == 9) // handle 0 --> 9 rollover
      {
        time[digit] = 0;
      }
      else
      {
        time[digit]++;
      }
    }

    // Zero the nixie data structs otherwise we turn on all elements at once
    cmhh = 0;
    cssm = 0;

    cmhh |= (unsigned long) NixieArray[time[2]] << 20 
          | (unsigned long) NixieArray[time[1]] << 10 
          | (unsigned long) NixieArray[time[0]];
    cssm |= (unsigned long) NixieArray[time[5]] << 20 
          | (unsigned long) NixieArray[time[4]] << 10 
          | (unsigned long) NixieArray[time[3]];
    
    SPI.transfer(cssm>>24);
    SPI.transfer(cssm>>16);
    SPI.transfer(cssm>>8);
    SPI.transfer(cssm);

    SPI.transfer(cmhh>>24);
    SPI.transfer(cmhh>>16);
    SPI.transfer(cmhh>>8);
    SPI.transfer(cmhh);

    digitalWrite(LEpin, HIGH);
    _delay_ms(69); // this all happens in <1 sec if this number is under 100ms
    digitalWrite(LEpin, LOW);
  }

}

/**
 * @brief Fills the shift reg with the passed in time, using a mask. 
*/
void sendTime(const int *mask, const DateTime &now)
{
  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // asssemble the two 32 bit structures that will be pushed into shift registers
  
  /*
  * apply the mask to the time so that digits can be zeroed.
  * digits = {H, h, M, m, S, s}
  * mask = {H, h, M, m, S, s}
  * cmmh = {Cc M h H}
  * cssm = {Cc s S m}
  */ 
  
  int dots = DOT_MODE;
  if (DOT_MODE == 3)
  {
    dots = now.second() % 2;
  } 

  cmhh |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[now.minute() / 10] * mask[MIN_T]) << 20 
        | (unsigned long) (NixieArray[hours24to12(now.hour()) % 10] * mask[HOUR_O]) << 10 
        | (unsigned long) (NixieArray[hours24to12(now.hour()) / 10] * mask[HOUR_T]);
  cssm |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[now.second() % 10] * mask[SEC_O]) << 20 
        | (unsigned long) (NixieArray[now.second() / 10] * mask[SEC_T]) << 10 
        | (unsigned long) (NixieArray[now.minute() % 10] * mask[MIN_O]);

  // & send them out over SPI.
  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(cmhh>>24);
  SPI.transfer(cmhh>>16);
  SPI.transfer(cmhh>>8);
  SPI.transfer(cmhh);
}

/**
 * @brief Fills the shift reg with the passed in time. 
*/
void sendTime(const DateTime &now)
{
  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // asssemble the two 32 bit structures that will be pushed into shift registers

  // control dots appropriately
  int dots = DOT_MODE;
  if (DOT_MODE == 3)
  {
    dots = now.second() % 2;
  } 

  cmhh |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[now.minute() / 10]) << 20 
        | (unsigned long) (NixieArray[hours24to12(now.hour()) % 10]) << 10 
        | (unsigned long) (NixieArray[hours24to12(now.hour()) / 10]);
  cssm |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[now.second() % 10]) << 20 
        | (unsigned long) (NixieArray[now.second() / 10]) << 10 
        | (unsigned long) (NixieArray[now.minute() % 10]);

  // & send them out over SPI.
  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(cmhh>>24);
  SPI.transfer(cmhh>>16);
  SPI.transfer(cmhh>>8);
  SPI.transfer(cmhh);
}

/**
 * @brief Fills the shift reg with the passed in array of digits, using the provided mask.
*/
void sendDigits(const int *mask, int dots, const int *digits)
{
  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // asssemble the two 32 bit structures that will be pushed into shift registers
  
  /*
  * apply the mask to the time so that digits can be zeroed.
  * digits = {H, h, M, m, S, s}
  * mask = {H, h, M, m, S, s}
  * cmmh = {Cc M h H}
  * cssm = {Cc s S m}
  */ 
 
  cmhh |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[digits[MIN_T]] * mask[MIN_T]) << 20 
        | (unsigned long) (NixieArray[digits[HOUR_O]] * mask[HOUR_O]) << 10 
        | (unsigned long) (NixieArray[digits[HOUR_T]] * mask[HOUR_T]);
  cssm |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[digits[SEC_O]] * mask[SEC_O]) << 20 
        | (unsigned long) (NixieArray[digits[SEC_T]] * mask[SEC_T]) << 10 
        | (unsigned long) (NixieArray[digits[MIN_O]] * mask[MIN_O]);

  // & send them out over SPI.
  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(cmhh>>24);
  SPI.transfer(cmhh>>16);
  SPI.transfer(cmhh>>8);
  SPI.transfer(cmhh);
}

/**
 * @brief Fills the shift reg with the passed in array of digits.
*/
void sendDigits(int dots, const int *digits)
{
  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // asssemble the two 32 bit structures that will be pushed into shift registers
 
  cmhh |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[digits[MIN_T]]) << 20 
        | (unsigned long) (NixieArray[digits[HOUR_O]]) << 10 
        | (unsigned long) (NixieArray[digits[HOUR_T]]);
  cssm |= (unsigned long) (dots) << 31 
        | (unsigned long) (dots) << 30
        | (unsigned long) (NixieArray[digits[SEC_O]]) << 20 
        | (unsigned long) (NixieArray[digits[SEC_T]]) << 10 
        | (unsigned long) (NixieArray[digits[MIN_O]]);

  // & send them out over SPI.
  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(cmhh>>24);
  SPI.transfer(cmhh>>16);
  SPI.transfer(cmhh>>8);
  SPI.transfer(cmhh);
}

/**
 * @brief Fills the shift reg with the passed in string (must be 6 ints, or x for digit off), does not light tubes. 
*/
void sendString(int dots, const char *str)
{
  unsigned long cmhh = 0; // data structure for colon 1, minute tens, hour ones, hour tens
  unsigned long cssm = 0; // data structure for colon 2, seconds ones, second tens, and minute ones

  // update colons mask
  cmhh |= (unsigned long) dots << 31 | (unsigned long)dots << 30;
  cssm |= (unsigned long) dots << 31 | (unsigned long)dots << 30;

  // blanks digits with an 'x' in the passed in string - this seems inefficient? Oh well...
  for (int i = 0; i < 6; i++)
  {
    if (str[i] == 'x')
    {
      cmhh |= (unsigned long) 0 << (10*(i%3));
      cssm |= (unsigned long) 0 << (10*(i%3));
    }
    else
    {
      cmhh |= (unsigned long) NixieArray[str[i] - '0'] << (10*(i%3));
      cssm |= (unsigned long) NixieArray[str[i] - '0'] << (10*(i%3));
    }
  }

  SPI.transfer(cssm>>24);
  SPI.transfer(cssm>>16);
  SPI.transfer(cssm>>8);
  SPI.transfer(cssm);

  SPI.transfer(cmhh>>24);
  SPI.transfer(cmhh>>16);
  SPI.transfer(cmhh>>8);
  SPI.transfer(cmhh);

}

/**
 * @brief Fades out of changing digits. Must apss in mask telling which ones are unchanging.
*/
void fadeTubes(const int *mask, const DateTime &dt)
{
  int fadeDuration = 1000;

  for (int pw = fadeDuration; pw > 0; pw = pw-5)
  { 
    sendTime(dt); // send the  time

    // TODO: Figure out how to do this in a non-blocking way. Perhaps hardware timers? Only cycling a pin so may be ok?
    digitalWrite(LEpin, HIGH);
    _delay_us(pw);
    digitalWrite(LEpin, LOW);

    sendTime(mask, dt); // send the time with mask applied

    digitalWrite(LEpin, HIGH);
    _delay_us(fadeDuration-pw);
    digitalWrite(LEpin, LOW);
  }
  _delay_us(fadeDuration);

}

/**
 * @brief Prints all data stored in eeprom over serial.
*/
void dumpEeprom()
{
  Serial.println("Dump all eeprom data: ");
  for (int i = 1; i <= 14; i++)
  {
    Serial.print("Data at EEPROM address ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(EEPROM.read((byte)i));
  }
  Serial.println("End of data.");

}