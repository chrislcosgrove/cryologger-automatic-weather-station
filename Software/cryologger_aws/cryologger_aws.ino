/*
    Title:    Cryologger Automatic Weather Station v0.1
    Date:     April 21, 2022
    Author:   Adam Garbo

    Description:
    - Code configured for automatic weather stations to be deployed
    in Arctic Bay and Pond Inlet.

    Components:
    - Rock7 RockBLOCK 9603
    - Maxtena M1621HCT-P-SMA antenna (optional)
    - Adafruit Feather M0 Proto
    - Adafruit Ultimate GPS Featherwing
    - Adafruit LSM303AGR Accelerometer/Magnetomter
    - Adafruit DPS310 Precision Barometric Pressure Sensor
    - Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3
    - Pololu 5V, 600mA Step-Down Voltage Regulator D36V6F5
    - Pololu 9V, 600mA Step-Down Voltage Regulator D36V6F9

    Sensors:
    - Vaisala HMP60 A12A0A3A0
    - RM Young Wind Monitor 5103L

    Comments:
    -

*/

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <Adafruit_DPS310.h>        // https://github.com/adafruit/Adafruit_DPS310 (v1.1.1)
#include <Adafruit_LSM303_Accel.h>  // https://github.com/adafruit/Adafruit_LSM303_Accel (v1.1.4)
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor (v1.1.4)
#include <Arduino.h>                // Required for new Serial instance. Include before <wiring_private.h>
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower (v1.2.2)
#include <IridiumSBD.h>             // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.1)
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero (v1.6.0)
#include <SAMD_AnalogCorrection.h>  // https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/SAMD_AnalogCorrection
#include <Statistic.h>              // https://github.com/RobTillaart/Statistic
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time (v1.6.1)
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus (v1.0.2b)
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // Required for creating new Serial instance

// ----------------------------------------------------------------------------
// Debugging macros
// ----------------------------------------------------------------------------
#define DEBUG           true  // Output debug messages to Serial Monitor
#define DEBUG_GNSS      true  // Output GNSS debug information
#define DEBUG_IRIDIUM   true  // Output Iridium debug messages to Serial Monitor

#if DEBUG
#define DEBUG_PRINT(x)            SERIAL_PORT.print(x)
#define DEBUG_PRINTLN(x)          SERIAL_PORT.println(x)
#define DEBUG_PRINT_HEX(x)        SERIAL_PORT.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x)      SERIAL_PORT.println(x, HEX)
#define DEBUG_PRINT_DEC(x, y)     SERIAL_PORT.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   SERIAL_PORT.println(x, y)
#define DEBUG_WRITE(x)            SERIAL_PORT.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
#define PIN_VBAT            A0  //
#define PIN_TEMP            0   //
#define PIN_HUMID           0   //
#define PIN_WIND_SPEED      0   //
#define PIN_WIND_DIR        0   //
#define PIN_IMU_EN          0   //
#define PIN_SENSOR_EN       0   //   
#define PIN_GNSS_EN         0   // 
#define PIN_5V_SW           0   // 
#define PIN_12V_SW          6   // 
#define PIN_IRIDIUM_RX      10  // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11  // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12  // Pin 7 OnOff (Grey)
#define PIN_MICROSD_CS      4   // MicroSD chip select pin

// ----------------------------------------------------------------------------
// Port configuration
// ----------------------------------------------------------------------------
// Create a new UART instance and assign it to pins 10 (RX) and 11 (TX).
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, PIN_IRIDIUM_RX, PIN_IRIDIUM_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);

#define SERIAL_PORT   Serial
#define GNSS_PORT     Serial1
#define IRIDIUM_PORT  Serial2

// Attach interrupt handler to SERCOM for new Serial instance
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// ----------------------------------------------------------------------------
// Object instantiations
// ----------------------------------------------------------------------------
Adafruit_DPS310                 dps310;   // I2C address: 0x77
Adafruit_LSM303_Accel_Unified   lsm303 = Adafruit_LSM303_Accel_Unified(54321); // I2C address: 0x??
IridiumSBD                      modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero                         rtc;
TinyGPSPlus                     gnss;

// ----------------------------------------------------------------------------
// Statistics objects
// ----------------------------------------------------------------------------
Statistic batteryStats;         // Battery voltage
Statistic extTemperatureStats;  // External temperature
Statistic extHumidityStats;     // External humidity
Statistic intTemperatureStats;  // Internal temperature
Statistic intHumidityStats;     // Internal humidity 
Statistic intPressureStats;     // Internal pressure 
Statistic windSpeedStats;       // Wind speed
Statistic vnStats;              // Wind north-south wind vector component (v)
Statistic veStats;              // Wind east-west wind vector component (u) 

// ----------------------------------------------------------------------------
// User defined global variable declarations
// ----------------------------------------------------------------------------
unsigned int  sampleInterval    = 60;   // Sleep duration (in seconds) between data sample acquisitions. Default = 5 minutes (300 seconds)
unsigned long alarmInterval     = 60;   // Sleep duration in seconds
unsigned int  transmitInterval  = 3;    // Messages to transmit in each Iridium transmission (340 byte limit)
unsigned int  retransmitLimit   = 2;    // Failed data transmission reattempt (340 byte limit)
unsigned int  gnssTimeout       = 1;    // Timeout for GNSS signal acquisition (minutes
unsigned int  iridiumTimeout    = 10;   // Timeout for Iridium transmission (s)
bool          firstTimeFlag     = true; // Flag to determine if the program is running for the first time

// ------------------------------------------------------------------------------------------------
// Global variable declarations
// ------------------------------------------------------------------------------------------------
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;      // Watchdog Timer interrupt counter
bool          resetFlag         = 0;      // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
unsigned int  iterationCounter  = 0;      // Counter for program iterations (zero indicates a reset)
byte          samples           = 30;     // Number of samples to average readings
byte          retransmitCounter = 0;      // Counter of Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;      // Counter of Iridium 9603 transmission intervals
unsigned int  failureCounter    = 0;      // Counter of consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;      // Global millis() timer
unsigned long alarmTime         = 0;      // Global epoch alarm time variable
unsigned long unixtime          = 0;      // Global epoch time variable

float         extTemperature    = 0.0;    // HMP60 temperature (°C)
float         intTemperature    = 0.0;    // DPS310 temperature (°C)
float         windSpeed         = 0.0;    // Wind speed (m/s)
float         windGust          = 0.0;    // Wind gust speed  (m/s)
unsigned int  windDirection     = 0;      // Wind direction (°)
float         windGustDirection = 0.0;    // Wind gust direction (°)
float         voltage           = 0.0;    // Battery voltage (V)

tmElements_t  tm;                         // Variable for converting time elements to time_t

// ----------------------------------------------------------------------------
// Unions/structures
// ----------------------------------------------------------------------------

// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages
typedef union
{
  struct
  {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   intTemperature;     // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t  intPressure;        // Interal pressure (hPa)         (2 bytes)   - 850 * 100
    int16_t   extTemperature;     // HMP60 temperature (°C)         (2 bytes)   * 100
    uint16_t  extHumidity;        // HMP60 humidity (%)             (2 bytes)   - 850 * 100

    int16_t   pitch;              // Pitch (°)                      (2 bytes)   * 100
    int16_t   roll;               // Roll (°)                       (2 bytes)   * 100
    uint16_t  windSpeed;          // Mean wind speed (m/s)          (2 bytes) (windSpeed * 100)
    uint16_t  windDirection;      // Mean wind direction (°)        (2 bytes)
    uint16_t  windGust;           // Wind gust speed (m/s)          (2 bytes) (windGust * 100)
    uint16_t  windGustDirection;  // Wind gust direction (°)        (2 bytes)
    
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  hdop;               // HDOP                           (2 bytes)
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)   * 100
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint8_t   transmitStatus;     // Iridium return code            (1 byte)
    uint16_t  iterationCounter;   // Message counter                (2 bytes)
  } __attribute__((packed));                              // Total: (32 bytes)
  uint8_t bytes[70];
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE moSbdMessage;

// Union to store received Iridium SBD Mobile Terminated (MT) message
typedef union
{
  struct
  {
    uint32_t  alarmInterval;      // 4 bytes
    uint8_t   transmitInterval;   // 1 byte
    uint8_t   retransmitLimit;    // 1 byte
    uint8_t   resetFlag;          // 1 byte
  };
  uint8_t bytes[7]; // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;

// Structure to store device online/offline states
struct struct_online
{
  bool dps310 = false;
  bool lsm303 = false;
  bool gnss = false;
} online;

// Union to store loop timers`
struct struct_timer
{
  unsigned long rtc;
  unsigned long battery;
  unsigned long dps310;
  unsigned long lsm303;
  unsigned long hmp60;
  unsigned long gnss;
  unsigned long iridium;
} timer;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_5V_SW, OUTPUT);
  pinMode(PIN_12V_SW, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_SENSOR_EN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_SENSOR_EN, LOW);   // Disable power to sensors
  digitalWrite(PIN_GNSS_EN, HIGH);    // Disable power to GNSS
  digitalWrite(PIN_5V_SW, LOW);       // Disable power to Iridium 9603
  digitalWrite(PIN_12V_SW, LOW);      // Disable power to Iridium 9603
  
  // Configure analog-to-digital (ADC) converter
  configureAdc();

  Wire.begin(); // Initialize I2C
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

#if DEBUG
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(4, 1000); // Non-blocking delay to allow user to open Serial Monitor
#endif

  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINTLN("Cryologger - Automatic Weather Station v1.0");
  printLine();

  // Configure devices
  configureRtc();       // Configure real-time clock (RTC)
  readRtc();            // Read date and time from RTC
  configureWdt();       // Configure Watchdog Timer (WDT)
  printSettings();      // Print configuration settings
  syncRtc();            // Synch RTC with GNSS
  configureIridium();   // Configure Iridium 9603 transceiver

  // Close serial port if immediately entering deep sleep
  if (!firstTimeFlag)
  {
    disableSerial();
  }

  // Blink LED to indicate completion of setup
  blinkLed(10, 100);
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
  // Check if RTC alarm triggered or if program is running for first time
  if (alarmFlag || firstTimeFlag)
  {
    // Read the RTC
    readRtc();

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      wakeUp();
    }

    DEBUG_PRINT("Info: Alarm trigger "); printDateTime();

    // Perform measurements
    petDog();         // Reset the Watchdog Timer
    readBattery();    // Read the battery voltage
    syncRtc();        // Sync the RTC with the GNSS
    readLsm303();     // Read the acceleromter
    readDps310();     // Read sensor(s)
    readHmp60();
    //readWind();
    writeBuffer();    // Write the data to transmit buffer
    transmitData();   // Transmit data via Iridium transceiver
    printTimers();    // Print function execution timers
    setRtcAlarm();    // Set the RTC alarm

    DEBUG_PRINTLN("Info: Entering deep sleep...");
    DEBUG_PRINTLN();

    // Prepare for sleep
    prepareForSleep();
  }

  // Check for Watchdog Timer interrupts
  if (wdtFlag)
  {
    petDog(); // Reset the Watchdog Timer
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}