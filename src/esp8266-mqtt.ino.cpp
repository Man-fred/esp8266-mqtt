/* 
 *  V01-00 : Arduino 1.6.7
 *  V01-01 : Arduino 1.6.10, DallasTemperature, OneWire updated
 *  V01-04 : Arduino 1.6.10, webserver for configdate, saved in eeprom, detection of NodeMCU and WeMos D1 mini
 *  V01-05 : Arduino 1.8.2,  
 *  V01-06 : Arduino 1.8.9,  
 *  V02-00-08: wie -07, aber Debuglevel 5 und mqtt feste IP
 *  V02-00-16: nach Exeptions in neuer Arduinoversion jetzt wieder stabil, IR, BH1745 und Neopixel deaktiviert
 *  V02-00-18: nach Exeptions in neuer Arduinoversion jetzt wieder stabil, IR, BH1745 und Neopixel aktiviert
 *  V03-00-01: PlatformIO, Arduino 1.8.13, ESP8266/ESP32, MQTT, OTA, Webserver, SPIFFS, LittleFS, EEPROM
 */
char mVersionNr[] = "V03-00-01.esp8266-mqtt.ino.";
/* Achtung: 1MB SPIFFS einstellen */

#define DEBUG 0

//#define USE_IR       // IR senden und empfangen

#define NEOPIXEL_KEYPAD 1
#define NEOPIXEL_AMBILIGHT 2
#define NEOPIXEL_COLOR 3
//#define USE_NEOPIXEL NEOPIXEL_COLOR  // 1: keypad 0..9; 2: ambilight; 3: fix color

//#define USE_BH1745   // RGB-Farb-Sensor
//#define USE_BH1750   // Helligkeitssensor
//#define USE_ADS1X15  // ADS1015/ADS1115 ADC

#define MAX_DS18B20 6 // Maximal 6 DS18B20 Sensoren
//#define MAX_DS2450 1 // Maximal 1 DS2450 ADC
#define USE_DS_SENSORS // OneWire

#ifndef DBG_OUTPUT_PORT
  #define DBG_OUTPUT_PORT Serial
#endif

#ifdef USE_NEOPIXEL
  #if USE_NEOPIXEL == NEOPIXEL_KEYPAD
    byte NEOPIXEL_COUNT = 10;  // Number of NeoPixels: Keypad 10, Test 1
  #else
    byte NEOPIXEL_COUNT = 120;
  #endif
#endif
#ifdef USE_ADS1X15
  #include "Adafruit_ADS1X15.h"
  Adafruit_ADS1115 ads[3];  /* Use this for the 16-bit version */
  //Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
#endif

#ifdef USE_DS_SENSORS
  #ifndef ESP32
    #include "OneWire.h"
    #ifdef MAX_DS18B20
      #include "DallasTemperature.h"
    #endif
    #ifdef MAX_DS2450
      #include "DS2450.h"
    #endif
  #endif
#endif

#include "arduino-crypto.h"
/*
   Wire - I2C Scanner

   The NodeMcu / WeMos D1 Mini I2C bus uses pins:
   D1 (5)= SCL
   D2 (4)= SDA
*/
/*
 *  Pin with LED on Wemos d1 mini D4 (2), Nodemcu D0 (16)
 */
/*
 * ARDUINO_* equals to ARDUINO_<...build.board from boards.txt>
 * OneWire-Bus D4 (2), collision with builtin LED on WeMos D1 Mini (2)
 * change to   D3 (0) 
 */
#ifndef ARDUINO_VARIANT
#define ARDUINO_VARIANT ARDUINO_BOARD
#endif
#ifdef ARDUINO_ESP8266_NODEMCU
  #include <ESP8266WiFi.h>
  // enables OTA updates
  #include <ESP8266httpUpdate.h>
  // enables webconfig
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #define PIN_MAX 9
  const byte board = 1;
  const byte Pin[PIN_MAX] = { D7, D6, D5, D8, D4, D1, D2, D0, D3 };  // reed3 oder S3
  char* PinName[] = { (char*)"D.7", (char*)"D.6", (char*)"D.5", (char*)"D.8", (char*)"D.4", (char*)"D.1", (char*)"D.2", (char*)"D.0", (char*)"D.3" };
  String mVersionBoard = "nodemcu";
#elif ARDUINO_ESP8266_WEMOS_D1MINI
  #include <ESP8266WiFi.h>
  // enables OTA updates
  #include <ESP8266httpUpdate.h>
  // enables webconfig
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #include "LittleFS.h"
  #define PIN_MAX 9
  const byte board = 2;
  const byte Pin[] = { D7, D6, D5, D0, D3, D1, D2, D4, D8 };
  char* PinName[] = { (char*)"D.7", (char*)"D.6", (char*)"D.5", (char*)"D.0", (char*)"D.3", (char*)"D.1", (char*)"D.2", (char*)"D.4", (char*)"D.8" };  //(char*) vor jedem String
  char mVersionBoard[] = "d1_mini";
#elif ARDUINO_NodeMCU_32S
  #define PIN_MAX 9
  #define WL_MAC_ADDR_LENGTH 8
  #include <WiFi.h>
  // enables OTA updates
  #include <HTTPUpdate.h>
  #include <HTTPClient.h>
  
  // enables webconfig
  #include <WebServer.h>
  #include <ESPmDNS.h>
  //#include <SPIFFS.h>
  #include "LittleFS.h" // LittleFS is declared
  #include <rom/rtc.h>
  
  const byte board = 3;
  #define D0 GPIO_NUM_16
  #define D1 GPIO_NUM_17
  #define D2 GPIO_NUM_18
  #define D3 GPIO_NUM_18
  #define D4 GPIO_NUM_19
  #define D5 GPIO_NUM_6
  #define D6 GPIO_NUM_22
  #define D7 GPIO_NUM_21
  #define D8 GPIO_NUM_8
  const byte Pin[] = { GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8 };  // D5: reed3 in oder S3 out
  char* PinName[] = { (char*)"D.16", (char*)"D.17", (char*)"D.18", (char*)"D.19", (char*)"D.6", (char*)"D.22", (char*)"D.21", (char*)"D.2", (char*)"D.8" };
  String mVersionBoard = "nodemcu-32s";
#elif DEVICE_ESP32_DEV
  #define PIN_MAX 9
  // #define BLE
  #define WL_MAC_ADDR_LENGTH 8
  #include <WiFi.h>
  // enables OTA updates
  #include <HTTPUpdate.h>
  #include <HTTPClient.h>
  
  // enables webconfig
  #include <WebServer.h>
  #include <ESPmDNS.h>
  //#include <SPIFFS.h>
  #include "LittleFS.h" // LittleFS is declared
  #include <rom/rtc.h>
  
  const byte board = 6;
  const byte Pin[] = { GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8 };  // D5: reed3 in oder S3 out
  char* PinName[] = { (char*)"D.16", (char*)"D.17", (char*)"D.18", (char*)"D.19", (char*)"D.6", (char*)"D.22", (char*)"D.21", (char*)"D.2", (char*)"D.8" };
  String mVersionBoard = "esp32";
#elif ARDUINO_d1_mini32
  #define PIN_MAX 9
  // #define BLE
  #define WL_MAC_ADDR_LENGTH 8
  //# define SYSTEM_EVENT_STA_LOST_IP 8
  //# define SYSTEM_EVENT_GOT_IP6 19
  #include <WiFi.h>
  // enables OTA updates
  #include <HTTPUpdate.h>
  #include <HTTPClient.h>
  
  // enables webconfig
  #include <WebServer.h>
  #include <ESPmDNS.h>
  //#include <SPIFFS.h>
  #include "LittleFS.h" // LittleFS is declared
  #include <rom/rtc.h>
  
  const byte board = 7;
  #define D0 GPIO_NUM_26
  #define D1 GPIO_NUM_22 // i2c CL
  #define D2 GPIO_NUM_21 // i2c DA
  #define D3 GPIO_NUM_17
  #define D4 GPIO_NUM_16
  #define D5 GPIO_NUM_18 // spi SCK
  #define D6 GPIO_NUM_19 // spi MISO
  #define D7 GPIO_NUM_23 // spi MOSI 
  #define D8 GPIO_NUM_5  // spi CS
  const byte Pin[] = { GPIO_NUM_26, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_5 };  // D5: reed3 in oder S3 out
  char* PinName[] = { (char*)"D.26", (char*)"D.22", (char*)"D.21", (char*)"D.17", (char*)"D.16", (char*)"D.18", (char*)"D.19", (char*)"D.23", (char*)"D.5" };
  String mVersionBoard = "esp32";
#elif ESP32
  #define PIN_MAX 9
  // #define BLE
  #define WL_MAC_ADDR_LENGTH 8
  //# define SYSTEM_EVENT_STA_LOST_IP 8
  //# define SYSTEM_EVENT_GOT_IP6 19
  #include <WiFi.h>
  // enables OTA updates
  #include <HTTPUpdate.h>
  #include <HTTPClient.h>
  
  // enables webconfig
  #include <WebServer.h>
  #include <ESPmDNS.h>
  //#include <SPIFFS.h>
  #include "LittleFS.h" // LittleFS is declared
  #include <rom/rtc.h>
  
  const byte board = 4;
  #define D0 GPIO_NUM_26
  #define D1 GPIO_NUM_22 // i2c CL
  #define D2 GPIO_NUM_21 // i2c DA
  #define D3 GPIO_NUM_17
  #define D4 GPIO_NUM_16
  #define D5 GPIO_NUM_18 // spi SCK
  #define D6 GPIO_NUM_19 // spi MISO
  #define D7 GPIO_NUM_23 // spi MOSI 
  #define D8 GPIO_NUM_5  // spi CS
  const byte Pin[] = { GPIO_NUM_26, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_5 };  // D5: reed3 in oder S3 out
  char* PinName[] = { (char*)"D.26", (char*)"D.22", (char*)"D.21", (char*)"D.17", (char*)"D.16", (char*)"D.18", (char*)"D.19", (char*)"D.23", (char*)"D.5" };
  String mVersionBoard = "esp32";
#else
  #define PIN_MAX 9
  #define SYSTEM_EVENT_STA_LOST_IP 8
  #define SYSTEM_EVENT_GOT_IP6 19
  #include <WiFi.h>
  // enables OTA updates
  #include <HTTPUpdate.h>
  // enables webconfig
  #include <WebServer.h>
  #include <ESPmDNS.h>
  //#include <SPIFFS.h>
  #include "LittleFS.h" // LittleFS is declared
  const byte board = 5;
  const byte Pin[] = { GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8 };  // D5: reed3 in oder S3 out
  char* PinName[] = { "D.16", "D.17", "D.18", "D.19", "D.6", "D.22", "D.21", "D.2", "D.8" };
  String mVersionBoard = ARDUINO_VARIANT;
#endif

#if DEBUG > 2
#define DEBUG3_PRINT(x, ...) DBG_OUTPUT_PORT.print(x, ##__VA_ARGS__)
#define DEBUG3_PRINTLN(x, ...) DBG_OUTPUT_PORT.println(x, ##__VA_ARGS__)
#define DEBUG3_PRINTF(x, ...) DBG_OUTPUT_PORT.printf(x, ##__VA_ARGS__)
#else
#define DEBUG3_PRINT(x, ...)
#define DEBUG3_PRINTLN(x, ...)
#define DEBUG3_PRINTF(x, ...)
#endif
#if DEBUG > 1
#define DEBUG2_PRINT(x, ...) DBG_OUTPUT_PORT.print(x, ##__VA_ARGS__)
#define DEBUG2_PRINTLN(x, ...) DBG_OUTPUT_PORT.println(x, ##__VA_ARGS__)
#define DEBUG2_PRINTF(x, ...) DBG_OUTPUT_PORT.printf(x, ##__VA_ARGS__)
#else
#define DEBUG2_PRINT(x, ...)
#define DEBUG2_PRINTLN(x, ...)
#define DEBUG2_PRINTF(x, ...)
#endif
#if DEBUG > 0
#define DEBUG1_PRINT(x, ...) DBG_OUTPUT_PORT.print(x, ##__VA_ARGS__)
#define DEBUG1_PRINTLN(x, ...) DBG_OUTPUT_PORT.println(x, ##__VA_ARGS__)
#define DEBUG1_PRINTF(x, ...) DBG_OUTPUT_PORT.printf(x, ##__VA_ARGS__)
#else
#define DEBUG1_PRINT(x, ...)
#define DEBUG1_PRINTLN(x, ...)
#define DEBUG1_PRINTF(x, ...)
#endif
#define DEBUG_PRINT(x, ...) DBG_OUTPUT_PORT.print(x, ##__VA_ARGS__)
#define DEBUG_PRINTLN(x, ...) DBG_OUTPUT_PORT.println(x, ##__VA_ARGS__)
#define DEBUG_PRINTF(x, ...) DBG_OUTPUT_PORT.printf(x, ##__VA_ARGS__)

#define MAX_MESSAGES 40
#define MQTT_MAX_TOPIC_SIZE 50

String fillNtpTime(unsigned long epoch);

boolean bleScanActive = true;

byte ONE_WIRE_BUS = Pin[4];
byte sclPin = Pin[5];
byte sdaPin = Pin[6];
byte ledPin = Pin[7];  //BUILTIN_LED;
byte ledPinOn = 0;
byte pwmPin = 0;
byte pwmPinOn = 0;

#ifdef USE_NEOPIXEL
  #include <string>
  byte pixelPin = 0xFF;  // Digital IO pin connected to the NeoPixels, set later
  byte pixelChanged = 0;
  byte pixelType = USE_NEOPIXEL;
  #include <Adafruit_NeoPixel.h>
  // Declare our NeoPixel strip object:
  Adafruit_NeoPixel strip(NEOPIXEL_COUNT, -1, NEO_GRB + NEO_KHZ800);
  // Argument 1 = Number of pixels in NeoPixel strip
  // Argument 2 = Arduino pin number (most are valid)
  // Argument 3 = Pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
#endif

#define PIXEL_ON 1
#define PIXEL_ALARM 2
#define PIXEL_SET 3
#define PIXEL_NA 4

#ifdef USE_IR
  #include <IRremoteESP8266.h>
  #include <IRrecv.h>
  #include <IRutils.h>
  #include <IRac.h>
  #include <IRtext.h>
  #include <IRsend.h>
  
  // An IR detector/demodulator is connected to GPIO pin 14(D5 on a NodeMCU
  // board).
  // Note: GPIO 16 won't work on the ESP8266 as it does not have interrupts.
  uint16_t kRecvPin = D4;  //14;
  IRrecv irrecv(kRecvPin);
  decode_results results;
  
  uint16_t kIrLed = D0;   //4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
  IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.
  
  void setup_ir(void) {
    irrecv.enableIRIn();  // Start the receiver
    irsend.begin();
  }
#endif

// enables storing webpages in EEPROM, not in sketch
#include <LittleFS.h>
// enables storing config-data in EEPROM
#include <EEPROM.h>
// enables NTP / RTC
#include <WiFiUdp.h>

#define MQTT_MAX_PACKET_SIZE 256  // default 128
#include <PubSubClient.h>
#include <Wire.h>
//#include <BMP280.h>
#include <Adafruit_BME280.h>
#ifdef USE_BH1750
  #include <BH1750FVI.h>
#endif
#include <PCF8574.h>

#ifdef BLE
  #include <BLEDevice.h>
#endif

#ifdef DS_MAX_SENSORS
// Fehler in der Implementierung, Absturz bei ESP32
  #ifndef ESP32
    #include <OneWire.h>
    #include <DallasTemperature.h>
    #include <DS2450.h>
  #endif
#endif

struct Parameter {
  unsigned char pVersion;
  char ssid[20];
  char password[20];

  char mqtt_server[20];
  int mqtt_port;
  char mqtt_user[20];
  char mqtt_pass[20];

  uint8_t mMac[6] = { 0, 0, 0, 0, 0, 0 };
  char mClient[20];

  char mPre[10];
  char mSub[10];
  char mLwt[10];
  unsigned int pVersion2;

  int timerMsec[3];
  byte pin[PIN_MAX];  // 0: inaktiv, 1: Sensor, 2: Schalter, 3: Alarm, 4: PWM
  byte GpioOn[PIN_MAX];
  byte GpioLedOn;
  int analog;
  char mKeypad[25];
  bool i2c;
  bool onewire;
  byte neopixelCount;
  unsigned int checksum;
};
struct Parameter para;


/* Set the tm_t fields for the local time. */
struct NtpTime {
  unsigned long epoch, isdst, sec, min, hour, wday, leap, year, mon, yday, mday, startup, delta;
};
struct NtpTime ntpTime;

Adafruit_BME280 bmp;
boolean bmpActive = false;
boolean bmeActive = false;
uint32_t bmpTyp;
//byte bmpADDR;
double bmpT = 0;
double bmpP = 0;
double bmeH = 0;
unsigned long bmpTime;
//#define bmpP0 1013.25

boolean bh1750Active = false;
#ifdef USE_BH1750
  BH1750FVI bh1750;
  uint16_t bh1750Lux = 0;
  unsigned long bh1750Time;
#endif

PCF8574 pcf8574(0x20);
boolean pcf8574Active = false;
boolean adsActive[3] = {false,false,false};
#ifdef USE_DS_SENSORS
  #ifndef ESP32
    OneWire ds;       /* Ini oneWire instance */
    #ifdef MAX_DS18B20
      DallasTemperature ds18b20(&ds); /* Dallas Temperature Library für Nutzung der oneWire */
    #endif
    #ifdef MAX_DS2450
      DS2450 ds2450(&ds);
    #endif
  #endif

  boolean dsActive = false;
  int ds1820Sensors = 0;
  int ds2450Sensors = 0;
  byte dsAddr[MAX_DS18B20][8];
  float dsTemp[MAX_DS18B20];
  float dsAD[4];
  unsigned long dsTime[MAX_DS18B20];
#endif

//String mChipId = "";
//String mClient = "";
unsigned int mFlashSize = 0;
boolean inSetup = true;

int pinState[PIN_MAX];
int reedTimer[PIN_MAX];

int reedAlarmstate = 0;  // 0: not armed, 1-7: armed
int reedSensor = 0;      // 0: no action, 1: Gpio1, 2: Gpio2, 4: Gpio3, 8: Gpio4
int reedActor = 0;       // 0: no action, 1: internal, 2: light, 4: external sirene, 8: reserve
byte reedAlarmtoggle = 0;

#define PIN_NOTHING 0
#define PIN_SENSOR 1
#define PIN_ACTOR 2
#define PIN_SWITCH 3
#define PIN_PWM 4
#define PIN_ALARM 5
#define PIN_SDA 6
#define PIN_SCL 7
#define PIN_1WIRE 8
#define PIN_LED 9
#define PIN_PIXEL 10
#define PIN_IRRECV 11
#define PIN_IRSEND 12

//boolean SOn[] = {false, false, false, false};

int analogWert = 0;
int analogState = 0;  // Analog < 512 -> 0, > 511 -> 1

char mPayloadKey[MAX_MESSAGES][MQTT_MAX_TOPIC_SIZE];
char mPayloadValue[MAX_MESSAGES][MQTT_MAX_PACKET_SIZE];
boolean mPayloadRetain[MAX_MESSAGES];
int mPayloadQos[MAX_MESSAGES];
int mPayloadSet = 0;
int mPayloadPublish = 0;
#define cstrLen 50
char cstr[MQTT_MAX_PACKET_SIZE];
char cmessage[MQTT_MAX_PACKET_SIZE];
char cpart[MQTT_MAX_PACKET_SIZE];
void mqttSet(char* key, char* value, boolean retain = true, int qos = 0);
void callback(char* topic, byte* payload, unsigned int mLength);
void keyclick(boolean clicking, int freq, int period);
void setup1wire(boolean rescan);
void ads1x15_loop();;
void testPara();
void setupPinmode();
void restartDelay();
void macToEsp(const unsigned char* mac, char* buf, byte len);
void macToString(const unsigned char* mac, char* buf, byte len);

/////// http.ino
/* 
  FSWebServer - Example WebServer with SPIFFS backend for esp8266
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WebServer library for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you CD in that folder and run the following command:
  for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done
  
  access the sample web page at http://esp8266fs.local
  edit the page by going to http://esp8266fs.local/edit
*/
//========= Variables for sketch-part "Http" ===============
WiFiClient espClient;
#ifdef ESP32
  WebServer http(80);
#else
  ESP8266WebServer http(80);
#endif
//holds the current upload
File fsUploadFile;
PubSubClient client(espClient);
boolean wifiStation = false;
boolean wifiAP = false;
boolean mqttConnected = false;

#define MAX_UNSIGNED_LONG 4294967295
class MyTimer {
  // Class Member Variables
  // These are initialized at startup
  unsigned int aPeriod;
  boolean aActive;
  boolean aSingle;
  void (*aCallback)();
  unsigned long previousMillis;  // will store last callback-time
  unsigned long currentMillis;

  // Constructor - creates a Flasher
  // and initializes the member variables and state
public:
  // MyTimer();

  void begin(int period, void (*func)(), boolean activated = true, boolean single = false) {
    aPeriod = period;
    aCallback = func;
    aSingle = single;
    activate(activated);
  }

  void activate(boolean is_active = true) {
    if ((aActive = is_active)) {
      if (aSingle) {
        previousMillis = millis();
      } else {
        // max. unsigned int to start the timer immediately
        previousMillis = MAX_UNSIGNED_LONG;
      }
    }
  }

  boolean deactivate() {
    if (aActive) {
      aActive = false;
      return true;
    }
    return false;
  }

  void run_now() {
    if (aSingle)
      deactivate();
    previousMillis = currentMillis;  // Remember the time
    yield();
    aCallback();
  }

  void update() {
    if (aActive) {
      currentMillis = millis();
      // check to see if it's time to start callback
      // Ueberlauf abfangen
      if (currentMillis - previousMillis >= aPeriod) {
        run_now();
      } else if (currentMillis < previousMillis) {  // overflow
        unsigned long rest = MAX_UNSIGNED_LONG - previousMillis;
        if (currentMillis + rest >= aPeriod) {
          run_now();
        }
      }
    }
  }
};

MyTimer timerMqtt;          // mqtt delay
MyTimer timerSensors;       // collect 1wire/i2c sensors delay
MyTimer timerKeypadloop;    // collect i2c Keypad delay
MyTimer timerKeypadreject;  // reject, if keypad not used for more than 2 sec
MyTimer timerReconnect;     // reconnect wifi delay
MyTimer timerNtp;           // ntp-loop
MyTimer timerRestartDelay;  // restart delay in websession and in station mode
MyTimer timerAlarmloop;     // alarm sensors delay
MyTimer timerAlarmstate;    // blink LED for local alarm

const byte hex[17] = "0123456789ABCDEF";

char* tochararray(char* cvalue, int value) {
  itoa(value, cvalue, 10);
  return cvalue;
}
char* tochararray(char* cvalue, unsigned int value, int radix = 10) {
  utoa(value, cvalue, radix);
  return cvalue;
}
char* tochararraycol(char* cvalue, unsigned int value) {
  int i = 6 - 1;
  while (i >= 0) {
    int rem = value % 16;
    cvalue[i--] = (rem > 9) ? (rem - 10) + 'A' : rem + '0';
    value = value / 16;
  }
  cvalue[6] = '\0';
  return cvalue;
}

char* tochararray(char* cvalue, float value, int len, int nachkomma) {
  dtostrf(value, len, nachkomma, cvalue);
  return cvalue;
}

char* tochararray(char* cvalue, char* value1, char value2) {
  strcpy(cvalue, value1);
  byte len = strlen(value1);
  cvalue[len] = hex[(value2 & 0xF0) >> 4];
  cvalue[len + 1] = hex[value2 & 0x0F];
  cvalue[len + 2] = '\0';
  return cvalue;
}
char* tochararray(char* cvalue, char* value1, char* value2) {
  strcpy(cvalue, value1);
  if (value2[0] != '\0')
    strcat(cvalue, value2);
  cvalue[strlen(value1) + strlen(value2)] = '\0';
  return cvalue;
}
char* tochararray(char* cvalue, char* value1) {
  strcpy(cvalue, value1);
  return cvalue;
}
char* tochararray(char* cvalue, String value1) {
  value1.toCharArray(cvalue, MQTT_MAX_PACKET_SIZE);
  return cvalue;
}
char* tochararray(char* cvalue, String value1, String value2) {
  char cpart[MQTT_MAX_PACKET_SIZE];
  value1.toCharArray(cvalue, MQTT_MAX_PACKET_SIZE);
  value2.toCharArray(cpart, MQTT_MAX_PACKET_SIZE);
  strcat(cvalue, cpart);
  return cvalue;
}

//========= Variables for sketch-part "Http" ===============

const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><input type='submit' value='Upload'></form><p><a href=\"/home.htm\">Goto Home</a></p>";
/* Called if requested page is not found */
void http404(){
  String response_message = "<html><head><title>ESP8266 Webserver</title></head><body style=\"background-color:PaleGoldenRod\"><h1><center>File Not Found</center></h1>";
  
  if (WiFi.status() == WL_CONNECTED)
  {
    response_message += "Status: Connected<br>";
  }
  else
  {
    response_message += "Status: Disconnected<br>";
  }
  response_message += "<a href=\"/home.htm\">Goto Home</a> <a href=\"/upload\">Upload files</a>";
  
  response_message += "URI: ";
  response_message += http.uri();
  response_message += "\nMethod: ";
  response_message += (http.method() == HTTP_GET)?"GET":"POST";
  response_message += "\nArguments: ";
  response_message += http.args();
  response_message += "\n";
  
  for (uint8_t i = 0; i < http.args(); i++)
  {
    response_message += " " + http.argName(i) + ": " + http.arg(i) + "\n";
  }
  response_message += "</body></html>";
  
  http.send(404, "text/html", response_message);
}

//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(http.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".json")) return "application/json";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  else if(filename.endsWith(".manifest")) return "text/cache-manifest";
  return "text/plain";
}

bool handleFileRead(String path){
  if(path.endsWith("/")) path += "home.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(LittleFS.exists(pathWithGz) || LittleFS.exists(path)){
    if(LittleFS.exists(pathWithGz))
      path += ".gz";
    DEBUG2_PRINT("handleFileRead: " + path);
    File file = LittleFS.open(path, "r");
    /*size_t sent = */ http.streamFile(file, contentType);
    file.close();
    DEBUG2_PRINTLN(" closed");
    return true;
  }
  return false;
}
void handleFileUploadDirect() {
    HTTPUpload& upload = http.upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (!filename.startsWith("/")) filename = "/" + filename;
      DEBUG2_PRINT("handleFileUpload Start: "); DEBUG2_PRINTLN(filename);
      fsUploadFile = LittleFS.open(filename, "w");
      filename = String();
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (fsUploadFile) {
        fsUploadFile.write(upload.buf, upload.currentSize);
        DEBUG2_PRINT("handleFileUpload Data : "); DEBUG2_PRINTLN(upload.currentSize);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) {
        fsUploadFile.close();
        DEBUG2_PRINT("handleFileUpload End  : "); DEBUG2_PRINTLN(upload.totalSize);
      }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        DBG_OUTPUT_PORT.println("handleFileUpload Aborted");
    }
}

void handleFileUpload(){
  if(http.uri() != "/edit") return;
  HTTPUpload& upload = http.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    DEBUG2_PRINT("handleFileUpload Name: "); DEBUG2_PRINTLN(filename);
    fsUploadFile = LittleFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    DEBUG2_PRINT("handleFileUpload Data: "); DEBUG2_PRINTLN(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    DEBUG2_PRINT("handleFileUpload Size: "); DEBUG2_PRINTLN(upload.totalSize);
  }
}

void handleFileDelete(){
  if(http.args() == 0) return http.send(500, "text/plain", "BAD ARGS");
  String path = http.arg(0);
  DEBUG1_PRINTLN("handleFileDelete: " + path);
  if(path == "/")
    return http.send(500, "text/plain", "BAD PATH");
  if(!LittleFS.exists(path))
    return http404();
  LittleFS.remove(path);
  http.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(http.args() == 0)
    return http.send(500, "text/plain", "BAD ARGS");
  String path = http.arg(0);
  DEBUG2_PRINTLN("handleFileCreate: " + path);
  if(path == "/")
    return http.send(500, "text/plain", "BAD PATH");
  if(LittleFS.exists(path))
    return http.send(500, "text/plain", "FILE EXISTS");
  File file = LittleFS.open(path, "w");
  if(file)
    file.close();
  else
    return http.send(500, "text/plain", "CREATE FAILED");
  http.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!http.hasArg("dir")) {
    http.send(500, "text/plain", "BAD ARGS"); 
    return;
  }
#ifdef ESP32
    http.send(500, "text/plain", "NOT ON ESP32"); 
#else
  String path = http.arg("dir");
  DEBUG2_PRINTLN("handleFileList: " + path);
  Dir dir = LittleFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\",\"size\":\"";
    output += String(entry.size());
    output += "\"}";
    entry.close();
  }
  output += "]";
  http.send(200, "text/json", output);
#endif
}

String getConfig(){
    byte i;
    String json = "{";
    json += "\"pVersion\":\""+String(para.pVersion + 1);
    json += "\",\"ssid\":\""+ String(para.ssid);
    json += "\",\"password\":\""+ String(para.password);
    json += "\",\"mqtt_server\":\""+ String(para.mqtt_server);
    json += "\",\"mqtt_port\":\""+ String(para.mqtt_port);
    json += "\",\"mqtt_user\":\""+ String(para.mqtt_user);
    json += "\",\"mqtt_pass\":\""+ String(para.mqtt_pass);
    json += "\",\"mClient\":\""+ String(para.mClient);
    json += "\",\"mPre\":\""+ String(para.mPre);
    json += "\",\"mSub\":\""+ String(para.mSub);
    json += "\",\"mLwt\":\""+ String(para.mLwt);
    for (i = 0; i < strlen(para.mKeypad); i++){
      if (!isgraph(para.mKeypad[i])) para.mKeypad[i] = '.';
    }
    json += "\",\"mKeypad\":\""+ String(para.mKeypad);
    for (i = 0; i < 3; i++) {
      json += "\",\"timerMsec"+String(i)+"\":\""+ String(para.timerMsec[i]);
    }
    for (i = 0; i < PIN_MAX; i++) {
      json += "\",\"PinName"+String(i)+"\":\""+ PinName[i];
      json += "\",\"pin"+String(i)+"\":\""+ String(para.pin[i]);
      json += "\",\"GpioOn"+String(i)+"\":\""+ String(para.GpioOn[i]);
    }
    json += "\",\"analog\":\""+ String(para.analog);
    json += "\",\"checksum\":\""+ String(para.checksum + 1);
    json += "\"}";
    return json;
}
String getIndex(){
    byte i;
    String json = "{";
    json += "\"myTitle\":\""+String(para.mClient);
    json += "\",\"myStatus\":\"";
    json += (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
    json += "\",\"heap\":\""+String(ESP.getFreeHeap());
    if (para.analog > 0) json += "\",\"analogstate\":\""+String(analogWert);
    for (i = 0; i < PIN_MAX; i++) {
      json += "\",\"pinState"+String(i)+"\":\""+String(pinState[i]);
    }
    json += "\",\"reedSensor\":\""+String(reedSensor);
    json += "\",\"reedAlarmstate\":\""+String(reedAlarmstate);
    json += "\",\"reedActor\":\""+String(reedActor);
#ifndef ESP32
    json += "\",\"gpio\":\""+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
#endif
    json += "\",\"bmp\":\""+String(bmpActive);
    if (bmpActive) {
      if (bmeActive) {
        json += "\",\"bmeH\":\""+String(bmeH,2);
      }
      json += "\",\"bmpT\":\""+String(bmpT,2);
      json += "\",\"bmpP\":\""+String(bmpP,0);
      json += "\",\"bmpTime\":\""+fillNtpTime(bmpTime);
    }
    json += "\",\"bh1750\":\""+String(bh1750Active);
    #ifdef USE_BH1750
      if (bh1750Active) {
        json += "\",\"bh1750Lux\":\""+String(bh1750Lux);
        json += "\",\"bh1750Time\":\""+fillNtpTime(bh1750Time);
      }
    #endif
    #ifdef MAX_DS18B20
      json += "\",\"ds\":\""+String(ds1820Sensors > 0);
      if (ds1820Sensors > 0) {
        for (i = 0; i < MAX_DS18B20; i++) {
          if (dsAddr[i][0] == 0x10 || dsAddr[i][0] == 0x28) {
            json += "\",\"dsT"+String(i)+"\":\""+String(dsTemp[i], 2);
            json += "\",\"dsTime"+String(i)+"\":\""+fillNtpTime(dsTime[i]);
          }
        }
      }
    #endif
    json += "\"}";
    return json;
}

char* getSsid(){
    int ap_count = WiFi.scanNetworks(false, true);
    String ssid;
    uint8_t encryptionType;
    int32_t RSSI;
    uint8_t* BSSID;
    int32_t channel;
    bool isHidden = false;

    char * json = (char *) malloc (1000);
    strcpy (json, "[\n");

    char network[100];
    for (int i = 0; i < ap_count; i++)
    {
#ifndef ESP32
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel, isHidden);
#else
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel);
      #define ENC_TYPE_NONE WIFI_AUTH_OPEN
#endif
      sprintf(network, "{\"SSID\":\"%s\",\"channel\":\"%d\",\"RSSI\":\"%d\",\"encryption\":\"%s\",\"hidden\":\"%s\" }\n", ssid.c_str(), channel, RSSI, encryptionType == ENC_TYPE_NONE ? " " : "*", isHidden ? "hidden" : "");
      if (strlen(json) < 899)
        strcat(json, network);
    }
    strcat(json, "]");
    return(json);
}
  
  /* Root page for the webserver */
void httpHome() {
  // Check if there are any GET parameters
  if (http.hasArg("restart")) {
    // muss in timer, sonst http-Timeout und endlos-Reset
    timerRestartDelay.begin(5000, restartDelay, true, true);
    if(!handleFileRead("/restart.htm")) http404();
  } else {
    if(!handleFileRead("/home.htm")) http404();
  }
}

String httpRestart(){
  // muss in timer, sonst http-Timeout und endlos-Reset
  DEBUG1_PRINTLN("httpRestart started");

  timerRestartDelay.begin(5000, restartDelay);
  String json = "{\"result\":\"Restart ausgelöst, bitte warten\"}";
  return json;
}

//* WLAN page allows users to set the WiFi credentials 
void httpConfig(){
  // Check if there are any GET parameters
  if (http.hasArg("ssid"))
  {    
    EEPROM.begin(4096);
    para.pVersion = http.arg("pVersion").toInt();
    para.pVersion2 = 123456 + para.pVersion;
    strncpy( para.ssid, http.arg("ssid").c_str(), 20); para.ssid[20 - 1] = '\0';
    strncpy( para.password, http.arg("password").c_str(), 20); para.password[20 - 1] = '\0';
    
    strncpy( para.mqtt_server, http.arg("mqtt_server").c_str(), 20); para.mqtt_server[20 - 1] = '\0';
    para.mqtt_port = http.arg("mqtt_port").toInt();
    strncpy( para.mqtt_user, http.arg("mqtt_user").c_str(), 20); para.mqtt_user[20 - 1] = '\0';
    strncpy( para.mqtt_pass, http.arg("mqtt_pass").c_str(), 20); para.mqtt_pass[20 - 1] = '\0';
    strncpy( para.mClient, http.arg("mClient").c_str(), 20); para.mClient[20 - 1] = '\0';
    strncpy( para.mPre, http.arg("mPre").c_str(), 10); para.mPre[10 - 1] = '\0';
    strncpy( para.mSub, http.arg("mSub").c_str(), 10); para.mSub[10 - 1] = '\0';
    strncpy( para.mLwt, http.arg("mLwt").c_str(), 10); para.mLwt[10 - 1] = '\0';
    strncpy( para.mKeypad, http.arg("mKeypad").c_str(), 25); para.mKeypad[25 - 1] = '\0';
    para.timerMsec[0] = http.arg("timerMsec0").toInt();
    para.timerMsec[1] = http.arg("timerMsec1").toInt();
    para.timerMsec[2] = http.arg("timerMsec2").toInt();
    for (byte i = 0; i < PIN_MAX; i++) {
      para.pin[i] = http.arg("pin"+String(i)).toInt();
      para.GpioOn[i] = http.arg("GpioOn"+String(i)).toInt();
    }
    para.analog = http.arg("analog").toInt();
    para.checksum = 123456; // ending int read 
    // 512k: 3c000 - 3ffff
    // 1024k 7c000
    // 2048/4096: 7c000 (max Flashsize 428k) oder fc000 (max Flashsize 940k)
    // 4096: zusaetzlich 1fc000 - 3fbfff (>= 2048 KB)
    EEPROM.put(0, para);
    EEPROM.commit();            // EEPROM Schreiben
    EEPROM.end();
    Serial.println("Flash written");
    setupPinmode();
  }
  if(!handleFileRead("/home.htm")) http404();
}

void listSpiffs(){
    DBG_OUTPUT_PORT.setDebugOutput(true);
//#ifndef ESP32
    Dir dir = LittleFS.openDir("/");
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      DEBUG1_PRINTF("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    DEBUG1_PRINTF("\n");
//#endif
}
void httpSetup() {
#ifdef ESP32
  LittleFS.begin(true); // formatOnFail
#else
  LittleFS.begin();
#endif
  listSpiffs();  
  //SERVER INIT
  /* Set page handler functions */
  http.on("/",   httpHome);
  http.on("/restart.json", [](){
    http.send(200, "application/json",  httpRestart());
  });
  http.on("/setConfig", httpConfig);
  //list directory
  http.on("/list", HTTP_GET, handleFileList);
  //load editor
  http.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) http404();
  });
  //create file
  http.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  http.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  http.on("/edit", HTTP_POST, [](){ http.send(200, "text/plain", ""); }, handleFileUpload);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  http.onNotFound([](){
    if(!handleFileRead(http.uri()))
      http404();
  });

  http.on("/upload", HTTP_GET, []() {
      http.sendHeader("Connection", "close");
      http.sendHeader("Access-Control-Allow-Origin", "*");
      http.send(200, "text/html", serverIndex);
  });
  http.on("/upload", HTTP_POST, []() {
      http.send(200, "text/html", serverIndex);
  }, handleFileUploadDirect);

  //get heap status, analog input value and all GPIO statuses in one json call
  http.on("/index.json", HTTP_GET, [](){
    http.send(200, "application/json", getIndex());
  });

  //get SSIDs for config-page
  http.on("/ssid.json", HTTP_GET, [](){
    int ap_count = WiFi.scanNetworks(false, true);
    String ssid;
    uint8_t encryptionType;
    int32_t RSSI;
    uint8_t* BSSID;
    int32_t channel;
    bool isHidden = false;

    String json = "[";
    for (int i = 0; i < ap_count; i++)
    {
      if (i > 0)
        json += ",\n";  
#ifndef ESP32
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel, isHidden);
      String network = "{\"SSID\":\""+ssid+"\",\"channel\":\""+String(channel)+
            "\",\"RSSI\":\""+String(RSSI)+"\",\"encryption\":\""+(encryptionType == ENC_TYPE_NONE ? " " : "*")+
            "\",\"hidden\":\""+(isHidden ? "hidden" : "")+"\" }";
#else
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel);
      String network = "{\"SSID\":\""+WiFi.SSID(i)+"\",\"channel\":\""+String(WiFi.channel(i))+
      "\",\"RSSI\":\""+String(WiFi.RSSI(i))+"\",\"encryption\":\""+(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " " : "*") +
      "\",\"hidden\":\""+(isHidden ? "hidden" : "")+"\" }";
#endif
      json += network;
      DEBUG3_PRINTLN(network);
    }
    json += "]";
    http.send(200, "application/json", json);
  });

  //get active config for config-page
  http.on("/config.json", HTTP_GET, [](){
    http.send(200, "application/json", getConfig());
  });

  //get heap status, analog input value and all GPIO statuses in one json call
  http.on("/all", HTTP_GET, [](){
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
#ifndef ESP32
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
#endif
    json += "}";
    http.send(200, "text/json", json);
    json = String();
  });
  
  http.begin();
  
  DEBUG1_PRINTLN("HTTP server started");
}

////////// http.ino

////////// ntp.ino
//========= Variables for sketch-part "Ntp" ===============
// A UDP instance to let us send and receive packets over UDP1
WiFiUDP udp;
unsigned int localPort = 2390;  // local port to listen for UDP packets
/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
IPAddress timeServerIP;
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;      // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets
//========= Variables for sketch-part "Ntp" ===============

String fillNtpTime(unsigned long epoch) {
  // Tage seit 1.1.1970 
  unsigned long day = epoch/(24L*60*60);
  /* We now have the number of days since 1970, but need days since 1900 */
  /* There are 17 leap years prior to 1970. Although 1900 was not a leap year */
  /* RTCTIME time cannot represent days before 1970 so make it a leap year to avoid */
  /* the special case (1900 not a leap year, but 2000 is */
  /* 25568 = (70 * 365) + 18 */
  day += 25568;
  // Sekunden am aktuellen Tag
  unsigned long secs = epoch % (24L*60*60);
  ntpTime.sec = secs % 60;
  unsigned long mins = secs / 60;
  ntpTime.hour = mins / 60;
  ntpTime.min = mins % 60;
  ntpTime.wday = (day + 4) % 7;

  /* Calculate year and day within year */
  unsigned long year = (day / (4*365+1)) * 4;
  day %= (4*365+1);
  
  /* If day in year is greater than 365 then adjust so it is 365 or less (0 is Jan 1st) */
  if(day > 365)
  {
      year += ((day - 1) / 365);
      day = (day - 1) % 365;
  }
  
  /* Day of year 1st Jan is 0 */
  ntpTime.yday = day + 1;
  
  /* Not a leap year and is feb 29 or greater */
  /* Then add a day to account for feb has 28, not 29 days */
  /* this is important for the month calculation below */
  ntpTime.leap = (year & 3) && (day >= 59);
  if(ntpTime.leap == 1) {
      day++;
  }
  ntpTime.year = year + 1900;
  
  /* Day of month is what's left, but we want it to start from 1, not 0 */
  // day++;
  
  if (day > 334 + ntpTime.leap) {
    ntpTime.mon = 12;
    day -= 334;
  } else if (day > 304 + ntpTime.leap) {
    ntpTime.mon = 11;
    day -= 304;
  } else if (day > 273 + ntpTime.leap) {
    ntpTime.mon = 10;
    day -= 273;
  } else if (day > 243 + ntpTime.leap) {
    ntpTime.mon =  9;
    day -= 243;
  } else if (day > 212 + ntpTime.leap) {
    ntpTime.mon =  8;
    day -= 212;
  } else if (day > 181 + ntpTime.leap) {
    ntpTime.mon =  7;
    day -= 181;
  } else if (day > 151 + ntpTime.leap) {
    ntpTime.mon =  6;
    day -= 151;
  } else if (day > 120 + ntpTime.leap) {
    ntpTime.mon =  5;
    day -= 120;
  } else if (day >  90 + ntpTime.leap) {
    ntpTime.mon =  4;
    day -= 90;
  } else if (day >  59 + ntpTime.leap) {
    ntpTime.mon =  3;
    day -= 59;
  } else if (day >  31) {
    ntpTime.mon =  2;
    day -= 31;
  } else 
    ntpTime.mon =  1;

  ntpTime.mday = day;
  String result = (ntpTime.mday < 10 ? "0" : "") + String(ntpTime.mday) + "." + (ntpTime.mon < 10 ? "0" : "") + String(ntpTime.mon) + "." + String(ntpTime.year)+ " " + String(ntpTime.hour)+ ":";
  // In the first 10 minutes of each hour, we'll want a leading '0'
  if (ntpTime.min < 10 ) 
    result += "0";
  result += String(ntpTime.min)+ ":";
  if (ntpTime.sec < 10 )
    result += "0";
  result += String(ntpTime.sec);
  return(result);
}

void getNTP() {
  DEBUG1_PRINTLN("Starting UDP");
  udp.begin(localPort);
  DEBUG3_PRINT("Local port: ");
#ifndef ESP32
  DEBUG3_PRINTLN(udp.localPort());
#endif
  //get a random server from the NTPpool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  // send an NTP request to the time server at the given address
  DEBUG1_PRINTLN("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(timeServerIP, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  // wait to see if a reply is available
  delay(2000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    DEBUG1_PRINTLN("no packet yet");
  }
  else {
    DEBUG1_PRINT("packet received, length=");
    DEBUG1_PRINTLN(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    ntpTime.startup = millis()/1000;
    udp.stop();
    
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    DEBUG3_PRINT("Seconds since Jan 1 1900 = " );
    DEBUG3_PRINTLN(secsSince1900);

    // now convert NTP time into everyday time:
    DEBUG2_PRINT("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // timezone Berlin, ToDo for all
    epoch += 2*3600;
    // print Unix time:
    DEBUG2_PRINTLN(epoch);

    ntpTime.epoch = epoch;
    ntpTime.delta = ntpTime.epoch - ntpTime.startup;
    fillNtpTime(epoch);
    
    // print the hour, minute and second:
    DEBUG1_PRINT("The (UTC) time is ");       // UTC is the time at Greenwich Meridian (GMT)
    DEBUG1_PRINT(ntpTime.mday); 
    DEBUG1_PRINT('.');
    DEBUG1_PRINT(ntpTime.mon); 
    DEBUG1_PRINT('.');
    DEBUG1_PRINT(ntpTime.year); 
    DEBUG1_PRINT(' ');
    DEBUG1_PRINT(ntpTime.hour); // print the hour (86400 equals secs per day)
    DEBUG1_PRINT(':');
    if (ntpTime.min < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      DEBUG1_PRINT('0');
    }
    DEBUG1_PRINT(ntpTime.min); // print the minute (3600 equals secs per minute)
    DEBUG1_PRINT(':');
    if (ntpTime.sec < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      DEBUG1_PRINT('0');
    }
    DEBUG1_PRINTLN(ntpTime.sec); // print the second
  }
}


////////// ntp.ino

////////// ble.ino
#ifdef BLE
// https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/SampleAsyncScan.cpp
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//#define BLE_MAXLOOPS 20
#define BLE_SCANTIME 30 //sek. iBeacon Suchzeit und MQTT Berichtinterval
#define BLE_MAX_DEVICES 20
unsigned int scanLoops = 0; //je 20 loops (10 Min) wieder alle aktiven Beacons melden
BLEScan* pBLEScan ;
String devices[BLE_MAX_DEVICES];
String devicesLong[BLE_MAX_DEVICES];
int rssi[BLE_MAX_DEVICES];
esp_ble_addr_type_t addressType[BLE_MAX_DEVICES];
int oldState[BLE_MAX_DEVICES] = {0};
int newState[BLE_MAX_DEVICES] = {0}; // 0: frei, 1: erkannt, 2: verloren
boolean inBleScan = false;
byte bleChanged = 0;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DEBUG_PRINT("Server onConnect: ");
      DEBUG_PRINTLN(pServer->getConnId());
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DEBUG_PRINT("Server onDisconnect: ");
      DEBUG_PRINTLN(pServer->getConnId());
    }
};

/**
 * Callback for each detected advertised device.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    int j = -1;
    int k = -1;
    String Nachricht, Inhalt;
    Nachricht = advertisedDevice.getAddress().toString().c_str();
    for (int i = 0; i < BLE_MAX_DEVICES && k < 0; i++) {
      if (devices[i] == Nachricht) {
        k = i;
        rssi[i] = advertisedDevice.getRSSI();
        newState[i] = 1;
        DEBUG3_PRINTLN(Nachricht+devices[i]+": "+String(rssi[i]) + " ("+String(newState[i])+") "+String(i)+" "+devicesLong[i]);
      } else if (newState[i] == 0  && j < 0) {
        j = i;
      }
    }
    if (k < 0 && j >= 0) {
      DEBUG2_PRINTF("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      devices[j] = Nachricht;
      devicesLong[j] = advertisedDevice.toString().c_str();
      rssi[j] = advertisedDevice.getRSSI();
      //addressType[j] = advertisedDevice.
      newState[j] = 1;
      bleChanged++;
      DEBUG3_PRINTLN(Nachricht+devices[j]+": "+String(rssi[j]) + " ("+String(newState[j])+") "+String(j)+" "+devicesLong[j]);
      //mqttSet(tochararray(cstr, devices[j]), BLEUtils::buildHexData(nullptr, advertisedDevice.getPayload(), advertisedDevice.getPayloadLength()), false, 3); // MAC per MQTT senden
    }
    if (bleChanged > 0) {
      String Nachricht = "[ ";
      j=0; 
      for (int i = 0; i < BLE_MAX_DEVICES; i++) {
        if (newState[i] > 0){
          if (j>0)
            Nachricht += ",";
          j++;
          Nachricht += "{ \"";
          Nachricht += devices[i];
          Nachricht += "\":\"";
          if (rssi[i] <= 0){
            Nachricht += String(rssi[i]);
          }else {
            Nachricht += BLEUtils::buildHexData(nullptr, advertisedDevice.getPayload(), advertisedDevice.getPayloadLength());
          }
          Nachricht += "\" }";
        }
      }
      Nachricht += " ]";
      mqttSet("BLEdevices", tochararray(cmessage, Nachricht)); // MAC per MQTT senden
      bleChanged = 0;
    }
  }
};

/**
 * Callback invoked when scanning has completed.
 */
static void scanCompleteCB(BLEScanResults scanResults) {
  DEBUG1_PRINTF("We found %d devices\n", scanResults.getCount());
  scanLoops++;
  DEBUG1_PRINT(scanLoops);
  DEBUG1_PRINTLN(". Scan complete!");
  //pBLEScan->clearResults(); // Speicher freigeben wird bei Start gemacht
  //yield();
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if ((newState[i] > 0) && (newState[i] != oldState[i])) {
      oldState[i] = newState[i];
      if (newState[i] == 2){
        //mqttSet(tochararray(cstr, devices[i]), "0", false, 3); // MAC per MQTT senden
        newState[i] = 0;
        rssi[i] = 0;
        bleChanged++;
      }
    }
    if (newState[i] == 1){
      newState[i] = 2;
    }
  }
  inBleScan = false;
} // scanCompleteCB

  /* 
   *  union esp_ble_key_value_t enthaelt esp_ble_pid_keys_t
   *  struct esp_ble_bond_key_info_t enthaelt esp_ble_pid_keys_t
   *  struct esp_ble_pid_keys_t, Public Members
        esp_bt_octet16_t irk             The irk value
        esp_ble_addr_type_t addr_type    The address type
        esp_bd_addr_t static_addr        The static address
   *  struct ble_scan_result_evt_param scan_rst: mapping auf advertisedDevice
   #define ESP_BLE_ENC_KEY_MASK    (1 << 0)            relate to BTM_BLE_ENC_KEY_MASK in stack/btm_api.h 
                                                       Used to exchange the IRK key in the init key & response key
   #define ESP_BLE_ID_KEY_MASK     (1 << 1)            relate to BTM_BLE_ID_KEY_MASK in stack/btm_api.h 
                                                       Used to exchange the CSRK key in the init key & response key
   #define ESP_BLE_CSR_KEY_MASK    (1 << 2)            relate to BTM_BLE_CSR_KEY_MASK in stack/btm_api.h 
                                                       Used to exchange the link key(this key just used in the BLE & BR/EDR coexist mode) in the init key & response key
   #define ESP_BLE_LINK_KEY_MASK   (1 << 3)            relate to BTM_BLE_LINK_KEY_MASK in stack/btm_api.h 
  */
void blePrintIRK(){
  int dev_num = esp_ble_get_bond_device_num() + 2;

  //int dev_num = esp_ble_gap_get_whitelist_size();
  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) {
    DEBUG1_PRINT("key_mask: ");
    DEBUG1_PRINT(dev_list[i].bond_key.key_mask);
    esp_ble_pid_keys_t *deviceKey = &(dev_list[i].bond_key.pid_key);
    DEBUG1_PRINTF("pid key addr type = %d", deviceKey->addr_type);
    DEBUG1_PRINT(" Public address: ");
    DEBUG1_PRINTF(ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(dev_list[i].bd_addr));
    DEBUG1_PRINT("pid static_addr:");
    DEBUG1_PRINTF(ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(deviceKey->static_addr));
    DEBUG1_PRINTF("IRK %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", deviceKey->irk[0],deviceKey->irk[1],deviceKey->irk[2],deviceKey->irk[3],deviceKey->irk[4],deviceKey->irk[5],deviceKey->irk[6],deviceKey->irk[7],deviceKey->irk[8],deviceKey->irk[9],deviceKey->irk[10],deviceKey->irk[11],deviceKey->irk[12],deviceKey->irk[13],deviceKey->irk[14],deviceKey->irk[15]); //ESP_BT_OCTET16_LEN
  }
  free(dev_list);

}
void setupBLE(){
  DEBUG1_PRINTLN("Starte BLE Scanner");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  //pBLEScan->setActiveScan(true);   // Aktives Bluetooth Scannen
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  blePrintIRK();
    
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void bleLoop(){
  if (!inBleScan && bleScanActive){
    DEBUG1_PRINTLN("Scan started!");
    inBleScan = true;
    pBLEScan->start(BLE_SCANTIME, scanCompleteCB); // BLE Scanen asynchron
  }
    // notify changed value
    if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&value, 4);
        pCharacteristic->notify();
        value++;
        delay(300); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
#endif //BLE

////////// ble.ino

////////// WifiMqtt.ino
void wifiAPSetup()
{
  wifiStation = false;
  wifiAP = true;
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  DEBUG1_PRINT("WiFi AP ...");
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  //String AP_NameString = macToEsp(mac);
  /*char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);
  */
  // "ESPxxxxxx"
  char AP_NameChar[10]; 
  macToEsp(mac, AP_NameChar, 10);
  WiFi.softAP(AP_NameChar, "123456789");
  DEBUG_PRINT(AP_NameChar);
  DEBUG_PRINTLN(", pass 123456789 ok"); 
  delay(500);                   //Abwarten 0,5s
  DEBUG_PRINT("IP Adresse ");  //Ausgabe aktueller IP des Servers
  DEBUG_PRINTLN(WiFi.softAPIP());
}

void wifiStaSetup() {
  if (!wifiAP && WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
#ifndef ESP32
    WiFi.setSleepMode(WIFI_NONE_SLEEP);//(WIFI_LIGHT_SLEEP);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#endif
    ////////////////////////////// ??????????????? WiFi.disconnect();
    wifiStation = false;
    int i = 0;
    DEBUG1_PRINT("WiFi.status() ");
    DEBUG1_PRINTLN(WiFi.status());
    DEBUG1_PRINT("WiFi.begin() ");
    DEBUG1_PRINT(para.ssid);
    DEBUG1_PRINTLN(para.password);
    WiFi.begin(para.ssid, para.password);
    // 60 Sekunden auf WLan warten
    while (WiFi.status() != WL_CONNECTED && i++ < 20) {
      delay(600);
      DEBUG1_PRINT(".");
    }
    if (i >= 20){
      // Erreichbar über AP 192.168.4.1 für 5 Min., anschließend Neustart im bekannten WLAN
      wifiAPSetup();
      timerRestartDelay.begin(300000, restartDelay, true, true);
    } else {
      DEBUG1_PRINT(", IP: ");
      DEBUG1_PRINT(WiFi.localIP());
      DEBUG1_PRINT(", MAC: ");
      WiFi.macAddress(para.mMac);                   //get MAC address of interface
      char myMac[20]; 
      macToString(para.mMac, myMac, sizeof(myMac));
      DEBUG1_PRINT(myMac);
      DEBUG1_PRINT(", Client: ");
      DEBUG1_PRINT(para.mClient);
      DEBUG1_PRINTLN(" ok");
      // besser nicht automatisch WiFi.setAutoReconnect(true);
      wifiStation = true;
      mqttSet((char*)"IP", tochararray(cstr, WiFi.localIP().toString()));
      mqttSet((char*)"RSSI", tochararray(cstr, WiFi.RSSI()));
    }
  }
}

void wifiSetup(boolean station) {
  if (!wifiAP){
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_OFF);
  
    if (station){
      wifiStaSetup();
    } else {
      wifiAPSetup();
    }
  }
}

#ifndef ESP32
/* IPAddress ip;
   IPAddress mask;
   IPAddress gw; */
void onGotIP(const WiFiEventStationModeGotIP& event){
  mqttSet((char*)"connect2", tochararray(cstr, event.ip.toString()), false);
  wifiStation = true;

  /*Serial.print("Station connected, IP: ");
  Serial.println(WiFi.localIP());*/
};

void onDisconnect(const WiFiEventStationModeDisconnected& event){
  if (!wifiAP){
    mqttSet((char*)"connect", tochararray(cstr, (char*)"disconnected ",event.reason), false);
    if (WiFi.status() == WL_CONNECTED) {
      // See https://github.com/esp8266/Arduino/issues/5912
      ///////////////////////////// ???????????????????? WiFi.disconnect();
    }
    wifiStation = false;
    Serial.println("Station disconnected");
  }
};
#endif

void mqttSetup(){
  DEBUG1_PRINTLN(para.mqtt_server);
  client.setServer(para.mqtt_server, para.mqtt_port);
  client.setCallback(callback);
}
void mqttLoop(){
  if (!wifiAP){
    if (client.connected()) {
      client.loop();
    }
  }
}
/* MQTT connect-errors
    -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
    -3 : MQTT_CONNECTION_LOST - the network connection was broken
    -2 : MQTT_CONNECT_FAILED - the network connection failed
    -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
    0 : MQTT_CONNECTED - the cient is connected
    1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
    2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
    3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
    4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
    5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect
*/
void mqttFailed(){
  mqttConnected = false;
  DEBUG1_PRINT("failed, rc=");
  DEBUG1_PRINT(client.state());
  DEBUG1_PRINT(", ");
  DEBUG1_PRINT(para.mqtt_server);
  DEBUG1_PRINT(", ");
  DEBUG1_PRINTLN(para.mqtt_port);
  mqttSet((char*)"mqttConnected", tochararray(cstr, client.state()));
}

/*  String macID = String(mac[WL_MAC_ADDR_LENGTH - 3], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
*/
void macToEsp(const unsigned char* mac, char* buf, byte len) {
  //char buf[10];
  snprintf(buf, len, "ESP%02X%02X%02X", mac[3], mac[4], mac[5]);
  Serial.printf("ESP%02X%02X%02X", mac[3], mac[4], mac[5]);
  //return buf;
}
void macToString(const unsigned char* mac, char* buf, byte len) {
  //char buf[20];
  
  snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  //return String(buf);
}
void mqttReconnect() {
  // Loop until we're reconnected
  if (!wifiAP && !client.connected()) {
    // MQTT disconnected
    mqttFailed();
    if (!wifiStation) {
      // WiFi disconnected
      wifiStaSetup();
    }
    if (wifiStation){
      DEBUG1_PRINT("MQTT...");
      // Attempt to connect
      // boolean connect (clientID, username, password, willTopic, willQoS, willRetain, willMessage)
      String lSubscribe = para.mPre;
      lSubscribe += para.mClient;
      lSubscribe += "/";
      String lLwt = lSubscribe + para.mLwt;
      lSubscribe += para.mSub;
      if (client.connect(para.mClient, para.mqtt_user, para.mqtt_pass, lLwt.c_str(), 0, 1, "down")) {
        DEBUG1_PRINTLN(" ok");
        // ... and subscribe to topic
        client.subscribe(lSubscribe.c_str());
        DEBUG1_PRINTLN(lSubscribe);
        mqttConnected = true;
        //timer1.deactivate();
        mqttSet(para.mLwt, (char*)"up");
        if (inSetup){
#ifndef ESP32
          mqttSet((char*)"ResetReason", tochararray(cstr, ESP.getResetReason()));
          mqttSet((char*)"ResetInfo", tochararray(cstr, ESP.getResetInfo()));
          mqttSet((char*)"ChipId", tochararray(cstr, ESP.getChipId()));
#else
          mqttSet((char*)"ResetReason", tochararray(cstr, (int) rtc_get_reset_reason(0)));
#endif
          mqttSet((char*)"Heap", tochararray(cstr, ESP.getFreeHeap()));
          mqttSet((char*)"Version", tochararray(cstr, mVersionNr, mVersionBoard));
          inSetup = false;
        }
      }
    }
  }
}
//////////http.ino

// Parameters for WiFi and MQTT
#ifdef ESP32
//WiFiEventHandler stationGotIpHandler, stationDisconnectedHandler;
#else
WiFiEventHandler stationGotIpHandler, stationDisconnectedHandler;
#endif

/*
void getSystem() {
  DEBUG3_PRINT("LED-Pin ");
  DEBUG3_PRINTLN(ledPin);
  DEBUG3_PRINT("ONE_WIRE_BUS ");
  DEBUG3_PRINTLN(ONE_WIRE_BUS);
  DEBUG3_PRINT(__FILE__);
  DEBUG3_PRINT(" ");
  DEBUG3_PRINT(__DATE__);
  DEBUG3_PRINT(" ");
  DEBUG3_PRINTLN(__TIME__);
  mFlashSize =  (ESP.getFlashChipId() >> 16) & 0xff; // lucky for us, WinBond u.a. ID's their chips as a form that lets us calculate the size (13-16)
  if (mFlashSize  < 18) {
    mFlashSize =  1 << mFlashSize; // lucky for us, WinBond u.a. ID's their chips as a form that lets us calculate the size
  }
  DEBUG3_PRINT("Flashsize ");
  DEBUG3_PRINTLN(mFlashSize,HEX);
}
*/
void getPara() {

  EEPROM.begin(512);
  EEPROM.get(0, para);
  if (para.pVersion > 0 && (para.pVersion2 == ((u32_t)123456 + (u32_t)para.pVersion))) {
    DEBUG1_PRINTLN("Flash loaded");
    testPara();
    //Notfallsystem, MQTT übersteuern: strncpy( para.mqtt_server, "192.168.178.126", 20); para.mqtt_server[20 - 1] = '\0';
  } else {
    // Default Parameter setzen //
    para.pVersion = 0;
    //strncpy(para.ssid, "...", 20);
    para.ssid[0] = '\0';

    strncpy(para.mqtt_server, "192.168.178.60", 20);
    para.mqtt_server[14] = '\0';
    para.mqtt_port = 1883;
    /*strncpy( para.mClient, "ESP", 20); para.mClient[20 - 1] = '\0';
    for (byte i = 3; i < 6; ++i) {
      para.mClient[2+(i*2)] = hex[(para.mMac[i] & 0xF0) >> 4];
      para.mClient[3+(i*2)] = hex[para.mMac[i] & 0x0F];
    }*/
#ifndef ESP32
    //strncpy(para.mClient, macToEsp(para.mMac), 10);
    macToEsp(para.mMac, para.mClient, 10);
#endif
    strncpy(para.mPre, "esp/", 10);
    para.mPre[10 - 1] = '\0';
    strncpy(para.mSub, "set/+", 10);
    para.mSub[10 - 1] = '\0';
    strncpy(para.mLwt, "lwt", 10);
    para.mLwt[10 - 1] = '\0';

    // timerMqtt: MQTT - Sendpipe
    para.timerMsec[0] = 1000;
    para.timerMsec[1] = 60000;
    para.timerMsec[2] = 20000;
    for (byte i = 0; i < PIN_MAX; ++i) {
      para.pin[i] = PIN_NOTHING;
      para.GpioOn[i] = 0;
    }
    para.pin[4] = PIN_1WIRE;
    para.pin[5] = PIN_SCL;
    para.pin[6] = PIN_SDA;
    para.pin[7] = PIN_LED;  //BUILTIN_LED;
    para.analog = 0;
    para.i2c = 0;
    para.onewire = 0;
    para.checksum = 111111;

    // 512k: 3c000 - 3ffff
    // 1024k 7c000
    // 2048/4096: 7c000 (max Flashsize 428k) oder fc000 (max Flashsize 940k)
    // 4096: zusaetzlich 1fc000 - 3fbfff (>= 2048 KB)
    EEPROM.put(0, para);
    EEPROM.commit();  // EEPROM Schreiben
    DEBUG1_PRINTLN("Flash initialized");
  }
  EEPROM.end();
}

void mqttSet(char* key, char* value, boolean retain, int qos) {
  byte payload = strlen(para.mPre) + strlen(para.mClient) + 1 + strlen(key);
  if (payload < MQTT_MAX_TOPIC_SIZE) {
    strncpy(mPayloadKey[mPayloadSet], para.mPre, strlen(para.mPre));
    strncat(mPayloadKey[mPayloadSet], para.mClient, strlen(para.mClient));
    strncat(mPayloadKey[mPayloadSet], "//", 1);
    strncat(mPayloadKey[mPayloadSet], key, strlen(key));
    mPayloadKey[mPayloadSet][payload] = '\0';
    payload = strlen(value);
    strncpy(mPayloadValue[mPayloadSet], value, payload);
    mPayloadValue[mPayloadSet][payload] = '\0';
    mPayloadRetain[mPayloadSet] = retain;
    mPayloadQos[mPayloadSet] = qos;
    DEBUG2_PRINT("mqttSet  ");
    DEBUG2_PRINT(mPayloadSet);
    DEBUG2_PRINT(": ");
    DEBUG2_PRINT(mPayloadKey[mPayloadSet]);
    DEBUG2_PRINT(" <");
    DEBUG2_PRINT(value);
    DEBUG2_PRINTLN(">");
    mPayloadSet++;
    if (mPayloadSet > MAX_MESSAGES - 1) {
      mPayloadSet = 0;
    }
    memset(mPayloadKey[mPayloadSet], 0, MQTT_MAX_TOPIC_SIZE);
  }
}

// Sends a payload to the broker
void mqttSend() {
  if (client.connected() /*&& mqttConnected*/ && mPayloadKey[mPayloadPublish][0] != '\0') {
    yield();
    int erg = client.publish(mPayloadKey[mPayloadPublish], mPayloadValue[mPayloadPublish], mPayloadRetain[mPayloadSet]);
    if (!erg && mPayloadQos[mPayloadPublish]-- > 0) {
      mqttSet(mPayloadKey[mPayloadPublish], mPayloadValue[mPayloadPublish], mPayloadRetain[mPayloadSet], mPayloadQos[mPayloadPublish]);
    }
    DEBUG2_PRINT("mqttSend ");
    DEBUG2_PRINT(mPayloadPublish);
    DEBUG2_PRINT(": ");
    DEBUG2_PRINT(mPayloadKey[mPayloadPublish]);
    DEBUG2_PRINT(" <");
    DEBUG2_PRINT(mPayloadValue[mPayloadPublish]);
    DEBUG2_PRINTLN(">");
    memset(mPayloadKey[mPayloadPublish], 0, MQTT_MAX_TOPIC_SIZE);
    mPayloadPublish++;
    if (mPayloadPublish > MAX_MESSAGES - 1) {
      mPayloadPublish = 0;
    }
  }
}

boolean bh1745Active = false;
uint8_t bh1745_brightness = 0xFF;
#ifdef USE_BH1745
  #include <BH1745NUC.h>
  BH1745NUC bh1745;
  uint32_t bh1745_color = 0;
  uint32_t bh1745_col2 = 0;
  uint16_t bh1745_red = 0;
  uint16_t bh1745_blue = 0;
  uint16_t bh1745_green = 0;
  uint16_t bh1745_lux = 0;

  boolean bh1745setup(void) {
    if (false){ //readRegister(bh1745.bh_i2cAddress, BH1745NUC_CMD_MANUFACTURER_ID) == 0xE0){
      bh1745.getAddr_BH1745NUC(BH1745NUC_DEFAULT_ADDRESS);  // 0x38, 0111000
      bh1745.setSWReset(SW_RESET_NOT_START);                // Initial reset is not started
      // bh1745.setSWReset(SW_RESET_START);                   // Initial reset is started
  
      bh1745.setMeasTime(MEAS_TIME_640);       // 010 : 640 msec
      bh1745.setRGBCValid(RGBC_VALID_UPDATE);  // RGBC data is updated after last writing MODE_CONTROL1,2 register or last reading MODE_CONTROL2 register
      bh1745.setRGBCEnable(RGBC_EN_ACTIVE);    // RGBC measurement is active
      // bh1745.setRGBCEnable(RGBC_EN_INACTIVE);              // RGBC measurement is inactive and becomes power down
  
      bh1745.setADCGain(ADC_GAIN_1X);                   // 00 : 1X
      bh1745.setINTStatus(STATUS_INACTIVE);             // Interrupt signal is inactive
      bh1745.setINTLatch(INT_LATCH_LATCHED);            // INTERRUPT pin is latched until INTERRUPT register is read or initialized
      bh1745.setINTEnable(INT_ENABLE_DISABLE);          // INTERRUPT pin disable
      bh1745.setPersistance(PERSISTENCE_UPDATED_EACH);  // Interrupt status is updated at each measurement end
      bh1745.Initialize();
      //delay(30);
      return true;  //this is only wire.begin(): bh1745.begin();
    } else {
      return false;
    }
  }
  void bh1745loop(void) {
    uint16_t red, green, blue;
    float f_red, f_blue, f_green, f_lux;

    // Read and print out the colors in lux values
    bh1745_red = bh1745.getRedColor();
    bh1745_green = bh1745.getGreenColor();
    bh1745_blue = bh1745.getBlueColor();
    bh1745_lux = bh1745.getClearColor();
/* AA3939 -> 170 57 57: gelesen 196 52 18 -> 0.86 1.09 3.16 
 * AA3939 -> 170 57 57: gelesen 240 59 30 -> 0.71 0.97 1.90 
 * AA3939 -> 170 57 57: gelesen 220 59 30 -> 0.77 0.97 1.90 
 */
    
    f_red = (float)bh1745_red * 0.77; // 1.39;
    f_green = (float)bh1745_green * 0.97; // 1.0
    f_blue = (float)bh1745_blue * 1.9; //1.75;
    f_lux = f_red > f_green ? f_red : f_green;
    f_lux = f_lux > f_blue ? f_lux : f_blue;
    if (f_lux > 255.0){
      f_red = f_red * 255.0 / f_lux;
      f_green = f_green * 255.0 / f_lux;
      f_blue = f_blue * 255.0 / f_lux;
    }
    DEBUG2_PRINT("RGBW: ");
    DEBUG2_PRINT(bh1745_red);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINT(bh1745_green);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINT(bh1745_blue);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINT(bh1745_lux);

    DEBUG2_PRINT(" rgb: ");
    DEBUG2_PRINT(f_red);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINT(f_green);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINT(f_blue);

    red = (uint8_t)f_red;
    green = (uint8_t)f_green;
    blue = (uint8_t)f_blue;
    bh1745_color = red << 16 | green << 8 | blue;
    bh1745_col2 = (byte)f_red << 16 | (byte)f_green << 8 | (byte)f_blue;
    DEBUG2_PRINT(" Color: ");
    DEBUG2_PRINT(bh1745_color, HEX);
    DEBUG2_PRINT(" ");
    DEBUG2_PRINTLN(bh1745_col2, HEX);
  #ifdef USE_NEOPIXEL
    if (pixelType == NEOPIXEL_AMBILIGHT) {
      for (byte i = 0; i < para.neopixelCount; i++) {
        strip.setPixelColor(i, red, green, blue);  //  Set pixel's color (in RAM)
      }
      strip.show();                      
    }
  #endif
  }

  void bh1745mqtt(void) {
    DEBUG_PRINT("Colorsensor RGB/Ambi-RGB/Lux: 0x");
    mqttSet((char*)"color", tochararraycol(cmessage, bh1745_color));
    DEBUG_PRINT(cmessage);
    DEBUG_PRINT("/");
    mqttSet((char*)"color2", tochararraycol(cmessage, bh1745_col2));
    DEBUG_PRINT(cmessage);
    DEBUG_PRINT("/");
    mqttSet((char*)"lux", tochararray(cmessage, bh1745_lux));
    DEBUG_PRINT("/");
    DEBUG_PRINTLN(cmessage);
    //mqttSet((char*)"red", tochararray(cmessage, bh1745_red, 16));
    //mqttSet((char*)"green", tochararray(cmessage, bh1745_green, 16));
    //mqttSet((char*)"blue", tochararray(cmessage, bh1745_blue, 16));
  }
#endif

#ifdef USE_BH1750
  void bh1750loop() {
    bh1750Lux = bh1750.readLightLevel();
    bh1750Time = ntpTime.delta + millis() / 1000;
    mqttSet((char*)"lux", tochararray(cmessage, bh1750Lux));

    DEBUG1_PRINT("Light: ");
    DEBUG1_PRINT(bh1750Lux);
    DEBUG1_PRINTLN(" lx");
  }
#endif

void bmp280loop() {
  bmp.takeForcedMeasurement();
  float t = bmp.readTemperature();
  if (abs(bmpT - t) > 0.009) {
    DEBUG1_PRINT("T = \t");
    DEBUG1_PRINT(bmpT, 2);
    DEBUG1_PRINT(" deg ");
    DEBUG1_PRINT(t, 3);
    bmpT = t;
    #ifdef DS_MAX_SENSORS
      if (ds1820Sensors > 0) {
        mqttSet((char*)"tempBMP", tochararray(cmessage, bmpT, 3, 1));
      } else {
    #endif
      mqttSet((char*)"temp", tochararray(cmessage, bmpT, 3, 1));
    #ifdef DS_MAX_SENSORS
      }
    #endif
  }
  float p = bmp.readPressure();
  if (abs(bmpP - p) > 9) {
    DEBUG1_PRINT("P = \t");
    DEBUG1_PRINT(bmpP / 100, 0);
    DEBUG1_PRINT(" mBar ");
    DEBUG1_PRINT(p / 100, 2);
    bmpP = p;
    mqttSet((char*)"pressure", tochararray(cmessage, float(bmpP / 100), 5, 1));
  }
  if (bmeActive) {
    float h = bmp.readHumidity();
    if (abs(bmeH - h) > 0.09) {
      DEBUG1_PRINT("H = \t");
      DEBUG1_PRINT(bmeH, 0);
      DEBUG1_PRINT(" % ");
      DEBUG1_PRINT(h, 2);
      bmeH = h;
      mqttSet((char*)"humidity", tochararray(cmessage, float(bmeH), 5, 1));
    }
  }
  DEBUG1_PRINTLN();
  /*
  char result = bmp.startMeasurment();

  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(bmpT, bmpP, bmeH);

    if (result > 0) {
      if (ds1820Sensors > 0) {
        mqttSet("tempBMP", String(bmpT, 2));
      } else {
        mqttSet("temp", String(bmpT, 2));
      }
      mqttSet("pressure", String(bmpP, 2));
      if (bmeActive) {
        mqttSet("humidity", String(bmeH, 2));
      }
      //double A = bmp.altitude(bmpP,P0);
      bmpTime = ntpTime.delta + millis() / 1000;

      DEBUG1_PRINT("T = \t"); DEBUG1_PRINT(bmpT, 2); DEBUG1_PRINT(" degC\t");
      DEBUG1_PRINT("P = \t"); DEBUG1_PRINT(bmpP, 2); DEBUG1_PRINT(" mBar\t");
      DEBUG1_PRINT("H = \t"); DEBUG1_PRINT(bmeH, 2); DEBUG1_PRINTLN(" %");
      //DEBUG1_PRINT("A = \t");DEBUG1_PRINT(A,2); DEBUG1_PRINTLN(" m");

    }
    else {
      DEBUG1_PRINT("BMP no result: ");
      DEBUG1_PRINTLN(result);
    }
  }
  else {
    DEBUG1_PRINTLN("BMP no Start.");
  }
  */
}

boolean dsSetup(boolean rescan) {
  #ifdef USE_DS_SENSORS
    #ifndef ESP32
      byte i, j;
      //byte present = 0;
      //byte data[12];
      String addr = "";
      ds.begin(ONE_WIRE_BUS);
      pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
      #ifdef MAX_DS18B20
        if (rescan){
          ds1820Sensors = 0;
        }
        if (!ds1820Sensors) {
          ds18b20.begin();
        }
        for (i = 0; i < MAX_DS18B20; i++) {
          if (!ds.search(dsAddr[i])) {
            if (i == 0) {
              DEBUG1_PRINTLN("No OneWire addresses.");
            }
            ds.reset_search();
            return (i > 0);
          } else {
            DEBUG2_PRINT("R=");
            addr = "";
            for (j = 0; j < 8; j++) {
              addr += String(dsAddr[i][j], HEX);
              addr += " ";
            }
            DEBUG2_PRINT(addr);
            DEBUG2_PRINT(" (");
            DEBUG2_PRINT(OneWire::crc8(dsAddr[i], 7));
            DEBUG2_PRINT(") ");

            if (OneWire::crc8(dsAddr[i], 7) != dsAddr[i][7]) {
              DEBUG2_PRINT("CRC nicht gültig!\n");
            } else if (dsAddr[i][0] == 0x10) {
              DEBUG2_PRINT("Sensor DS18S20\n");
              ds1820Sensors++;
              mqttSet(tochararray(cstr, (char*)"DS18S20", tochararray(cpart, i)), tochararray(cmessage, addr));
            } else if (dsAddr[i][0] == 0x28) {
              DEBUG2_PRINT("Sensor DS18B20\n");
              ds1820Sensors++;
              mqttSet(tochararray(cstr, (char*)"DS18B20", tochararray(cpart, i)), tochararray(cmessage, addr));
            } else if (dsAddr[i][0] == 0x20) {
              DEBUG2_PRINT("Sensor DS2450\n");
              #ifdef Max_DS2450
                if (!ds2450Sensors) {
                  ds2450.begin(dsAddr[i]);
                }
                ds2450Sensors++;
                mqttSet(tochararray(cstr, (char*)"DS2450", tochararray(cpart, i)), tochararray(cmessage, addr));
              #endif
            } else {
              DEBUG2_PRINT("Sensorfamilie nicht erkannt : 0x");
              DEBUG2_PRINTLN(dsAddr[i][0], HEX);
            }
          }
        }
      #endif
      return true;
    #else
      return false;
    #endif
  #else
    return false;
  #endif
}

void ds1820Loop() {
  #ifdef MAX_DS18B20  
    #ifndef ESP32
      int i;
      ds18b20.requestTemperatures();  // Temp abfragen
      for (i = 0; i < MAX_DS18B20; i++) {
        if (dsAddr[i][0] == 0x10 || dsAddr[i][0] == 0x28) {
          dsTemp[i] = ds18b20.getTempCByIndex(i);
          dsTime[i] = ntpTime.delta + millis() / 1000;
          char result[8];  // Buffer big enough for 7-character float
          char ds[6] = "temp0";
          dtostrf(dsTemp[i], 6, 1, result);  // Leave room for too large numbers!
          if (i == 0) {
            ds[4] = '\0';
            mqttSet(ds, result);
          } else {
            ds[4] = '0' + i;
            mqttSet(ds, result);
          }
          //DEBUG1_PRINT("temp"+String(i));
          //DEBUG1_PRINT(ds18b20.getTempCByIndex(i) );
          //DEBUG1_PRINTLN(" Grad Celsius");
        }
      }
    #endif
  #endif
}

void ds2450Loop() {
  #ifdef MAX_DS2450  
    #ifndef ESP32
      ds2450.update();
      if (ds2450.isError()) {
        DEBUG1_PRINT("Error reading from DS2450 device");
      } else {
        for (int channel = 0; channel < 4; channel++) {
          mqttSet(tochararray(cstr, (char*)"AD", tochararray(cpart, channel)), tochararray(cmessage, ds2450.getVoltage(channel), 4, 1));
          DEBUG1_PRINT("Channel ");
          DEBUG1_PRINT(char('A' + channel));
          DEBUG1_PRINT(" = ");
          DEBUG1_PRINT(ds2450.getVoltage(channel), 1);
          if (channel < 3) {
            DEBUG1_PRINT("V, ");
          } else {
            DEBUG1_PRINTLN("V.");
          }
        }
      }
    #endif
  #endif
}

void analogLoop() {
  analogWert = analogRead(A0);
  if (analogState == 0 && analogWert >= para.analog) {
    analogState = 1;
    mqttSet((char*)"analogIn", (char*)"1");
  } else if (analogState == 1 && analogWert < para.analog) {
    analogState = 0;
    mqttSet((char*)"analogIn", (char*)"0");
  }
}

void getData() {
  mqttSet((char*)"Heap", tochararray(cmessage, ESP.getFreeHeap()));
  yield();

  mqttSet((char*)"lwt", (char*)"up", true, 10);
  yield();

  if (bmpActive) {
    bmp280loop();
    yield();
  }
  #ifdef USE_BH1745
    if (bh1745Active) {
      bh1745mqtt();
      yield();
    }
  #endif
  #ifdef USE_BH1750
    if (bh1750Active) {
      bh1750loop();
      yield();
    }
  #endif
  #ifdef MAX_DS18B20
    if (ds1820Sensors > 0) {
      ds1820Loop();
      yield();
    }
  #endif
  #ifdef MAX_DS2450
    if (ds2450Sensors > 0) {
      ds2450Loop();
      yield();
    }
  #endif
  #ifdef USE_ADS1X15
    if (adsActive[0] || adsActive[1] || adsActive[2]){
      ads1x15_loop();
    }
  #endif
}

void setAlarmLED() {
  if ((reedAlarmtoggle++ & 7) == 1) {
    digitalWrite(ledPin, ledPinOn);
  } else {
    digitalWrite(ledPin, !ledPinOn);
  }
  //DEBUG1_PRINTLN("reedLED "+String(reedAlarmtoggle & 3)+" "+String(reedAlarmtoggle));
}

void setSwitch(byte nr, boolean set) {
  DEBUG1_PRINT("Set ");
  DEBUG1_PRINT(nr);
  if (pinState[nr] ^ set) {
    pinState[nr] = !pinState[nr];
    //mqttSet(("S"+String(nr)+"on").toChar(), tochararray(cstr, pinState[nr]));
    digitalWrite(Pin[nr], pinState[nr] ? para.GpioOn[nr] : (para.GpioOn[nr] == 0));
    DEBUG1_PRINT(" ");
    DEBUG1_PRINT(pinState[nr]);
  }
  DEBUG1_PRINT(" ");
  DEBUG1_PRINT(set);
  yield();
}

void setPwm(byte nr, byte set) {
  DEBUG_PRINTLN("test PWM " + String(nr) + " " + String(para.pin[nr]));
  if (para.pin[nr] == PIN_PWM) {
    switch (set) {
      case 0:
        keyclick(true, 0, 0);
        break;
      // Dauerton bis set == 0
      case 1:
        keyclick(true, 420, -1);
        break;

      // Bestätigung unscharf
      case 2:
        keyclick(true, 1260, 1000);
        break;

      // Bestätigung scharf
      case 3:
        keyclick(true, 840, 1000);
        break;

      // Bestätigung schaerfen
      case 4:
        keyclick(true, 220, 500);
        break;
    }
  }
}

void setAlarm() {
  if ((reedSensor & reedAlarmstate) > 0) {
    // start local alarm
    timerAlarmstate.activate();
  } else if (reedAlarmstate == 0) {
    timerAlarmstate.deactivate();
    digitalWrite(ledPin, !ledPinOn);
  }
  for (byte i = 0; i < PIN_MAX; i++) {
    if (para.pin[i] == PIN_ACTOR)
      setSwitch(i, reedActor & (1 << i));
  }
}

void Alarmloop() {
  int timer = millis();
  for (byte i = 0; i < PIN_MAX; i++) {
    if (para.pin[i] == PIN_SENSOR || para.pin[i] == PIN_ALARM) {
      int reedTemp = digitalRead(Pin[i]);
      if (reedTemp) {
        reedTemp = para.GpioOn[i];
      } else {
        reedTemp = para.GpioOn[i] == 0;
      }
      if ((reedTemp != pinState[i] && timer - reedTimer[i] > 100) | (timer - reedTimer[i] > 600000) | (timer < reedTimer[i]) | (reedTimer[i] == 0)) {
        reedTimer[i] = timer;
        pinState[i] = reedTemp;
        mqttSet(tochararray(cstr, (char*)"reed", tochararray(cpart, i + 1)), tochararray(cmessage, reedTemp));
        DEBUG1_PRINT("reed");
        DEBUG1_PRINT(i + 1);
        DEBUG1_PRINT(" ");
        DEBUG1_PRINTLN(reedTemp);
        if (reedTemp) {
          reedSensor = reedSensor | (1 << i);
        } else {
          reedSensor = reedSensor & ~(1 << i);
        }
        setAlarm();
      }
    }
  }
  yield();
  if (para.analog) {
    analogLoop();
  }
}

void restartDelay() {
  DEBUG1_PRINTLN("restartDelay started");
  ESP.restart();
}

void i2cScan(boolean mqtt = false) {
  byte error, address;
  int nDevices;
  char addr[3] = "00";
  char msg[37];
  if (!mqtt) DEBUG_PRINTLN("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    //Wire.requestFrom(address, 1);
    //_data = Wire.read();
    error = Wire.endTransmission();
    if (error == 0) {
      strcpy(msg, "I2C device found at address 0x     ");
      msg[33] = hex[(address & 0xF0) >> 4];
      msg[34] = hex[address & 0x0F];
      nDevices++;
    } else if (error == 4) {
      strcpy(msg, "Unknown error    at address 0x");
      msg[33] = hex[(error & 0xF0) >> 4];
      msg[34] = hex[error & 0x0F];
    }
    msg[30] = hex[(address & 0xF0) >> 4];
    msg[31] = hex[address & 0x0F];
    msg[35] = '\0';
    addr[0] = msg[30];
    addr[1] = msg[31];
    if (error == 0 || error == 4) {
      if (mqtt) {
        mqttSet(tochararray(cstr, (char*)"debug/i2c/", addr), msg, false);
      } else {
        DEBUG_PRINTLN(msg);
      }
    }
  }
  if (nDevices == 0) {
    strcpy(msg, "No I2C devices found");
    if (mqtt) {
      mqttSet((char*)"debug/i2c/00", msg, false);
    } else {
      DEBUG_PRINTLN(msg);
    }
  } else {
    if (!mqtt) DEBUG_PRINTLN("Done.\n");
  }
}

void setConfig(byte nr, char receivedChar) {
  if (nr == 1) {
    DEBUG1_PRINT(receivedChar);
    if (receivedChar == '0') {
      digitalWrite(ledPin, HIGH);
    }
    if (receivedChar == '1') {
      digitalWrite(ledPin, LOW);
    }
    if (receivedChar == '2') {
      timerMqtt.activate();
    }
    if (receivedChar == '3') {
      timerMqtt.deactivate();
    }
    if (receivedChar == '4') {
      restartDelay();
    }
    //OTA Update
    if (receivedChar == '5') {
      mqttSet((char*)"set/C1", (char*)"6");

      timerMqtt.deactivate();
      timerSensors.deactivate();
      timerKeypadloop.deactivate();
      boolean timerKeypadreject_active = timerKeypadreject.deactivate();
      timerReconnect.deactivate();
      timerNtp.deactivate();
      timerAlarmloop.deactivate();
#ifdef ESP32
      WiFiClient wifiClient;
      t_httpUpdate_return ret = httpUpdate.update(wifiClient, para.mqtt_server, 80, "/esp8266/ota.php", tochararray(cstr, mVersionNr, mVersionBoard));
#else
      WiFiClient wifiClient;
      t_httpUpdate_return ret = ESPhttpUpdate.update(wifiClient, para.mqtt_server, 80, "/esp8266/ota.php", tochararray(cstr, mVersionNr, mVersionBoard));
      //t_httpUpdate_return ret = ESPhttpUpdate.update(para.mqtt_server, 80, "/esp8266/ota.php", tochararray(cstr, mVersionNr, mVersionBoard));
#endif
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          DEBUG1_PRINTLN("[update] Update failed: ");
          mqttSet((char*)"Update", (char*)"failed");

          break;
        case HTTP_UPDATE_NO_UPDATES:
          DEBUG1_PRINTLN("[update] Update no Update.");
          mqttSet((char*)"Update", (char*)"not necessary");
          break;
        case HTTP_UPDATE_OK:
          DEBUG1_PRINTLN("[update] Update ok.");  // may not called we reboot the ESP
          mqttSet((char*)"Update", (char*)"ok");
          break;
        default:
          mqttSet((char*)"Update", tochararray(cmessage, ret));
          break;
      }
      timerMqtt.activate();
      timerSensors.activate();
      timerKeypadloop.activate();
      timerKeypadreject.activate(timerKeypadreject_active);
      timerReconnect.activate();
      timerNtp.activate();
      timerAlarmloop.activate();
    }
    if (receivedChar == '7') {
      mqttSet((char*)"set/C1", (char*)"6");
      LittleFS.format();
    }
  } else if (nr == 2) {
    if (receivedChar == 'i') {
      i2cScan(true);
    }
    if (receivedChar == 'r') {
      DEBUG1_PRINT("RSSI ");
      DEBUG1_PRINTLN(WiFi.RSSI());
    }
  }
  yield();
}

void setOSS(byte nr, char receivedChar) {

  DEBUG1_PRINT("OSS ");
  DEBUG1_PRINTLN(nr);
  if (nr == 0) {
    bmp.setSampling(Adafruit_BME280::MODE_NORMAL,  // Operating Mode.
                    Adafruit_BME280::SAMPLING_X1,  // Temp. oversampling
                    Adafruit_BME280::SAMPLING_X1,  // Pressure oversampling
                    Adafruit_BME280::SAMPLING_X1,  // Pressure oversampling
                    Adafruit_BME280::FILTER_OFF,   // Filtering.
                    Adafruit_BME280::STANDBY_MS_1000);
  }
  if (nr == 1) {
    bmp.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_0_5);
  }
  if (nr == 2) {
    bmp.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_1000);
  }
  yield();
}

void setArmed(boolean set) {
  reedAlarmstate = 0;
  DEBUG1_PRINT("Armed ");
  if (set) {
    for (byte i = 0; i < PIN_MAX; i++) {
      if (para.pin[i] == PIN_ALARM) {
        reedAlarmstate = reedAlarmstate | (1 << i);
      }
    }
  }
  DEBUG1_PRINTLN(reedAlarmstate);
  setAlarm();
  yield();
}

void setActor(byte nr, boolean set) {
  DEBUG1_PRINT("Actor ");
  if (set) {
    reedActor = reedActor | nr;
  } else {
    reedActor = reedActor & ~nr;
  }
  DEBUG1_PRINTLN(reedActor);
  setAlarm();
  yield();
}

void callbackOSS(byte* payload, unsigned int mLength) {
  if (mLength == 1 and payload[0] < 5) {
    //bmp.setOversampling(payload[0], 1, 1);
  }
}

void set(char request, byte nr, char payload) {
  if (request == 'C') {
    setConfig(nr, payload);
  } else if (request == 'O') {
    setOSS(nr, payload);
  } else if (request == 'S' && nr < PIN_MAX && para.pin[nr] == PIN_SWITCH) {
    setSwitch(nr, payload == '1');
  } else if (request == 'A' && nr < PIN_MAX && para.pin[nr] == PIN_ACTOR) {
    setActor(1 << nr, payload == '1');
  } else if (request == 'D') {
    setArmed(payload == '1');
  } else if (request == 'P' && nr < PIN_MAX) {
    setPwm(nr, payload - '0');
  }
}

void callback(char* topic, byte* payload, unsigned int mLength) {
  char receivedChar = (char)payload[0];
  cmessage[0] = '[';
  cmessage[1] = '\0';
  strcat(cmessage, topic);
  strcat(cmessage, "] ");
  int len = strlen(cmessage);
  cmessage[len] = receivedChar;
  cmessage[len + 1] = '\0';
  char slash = topic[strlen(topic) - 3];
  char request = topic[strlen(topic) - 2];
  byte nr = topic[strlen(topic) - 1] - '0';
  //DEBUG1_PRINT(slash);
  //DEBUG1_PRINT(request);
  //DEBUG1_PRINTLN(nr);
  //DEBUG1_PRINTLN(ack);
  if (request == 'I') {
    #ifdef USE_IR
      send_ir(nr, payload, mLength);
    #endif
  } else if (request == 'M') {
    #ifdef USE_NEOPIXEL    
      if (topic[strlen(topic) - 1] == 'A') {
        pixelType = NEOPIXEL_AMBILIGHT;
      } else if (topic[strlen(topic) - 1] == 'C') {
        if (mLength == 1 && payload[0] == 'A') {
          pixelType = NEOPIXEL_AMBILIGHT;
        } else {
          pixelType = NEOPIXEL_COLOR;
          pixelRgb(0, payload, mLength, NEOPIXEL_COUNT - 1);
        }
      } else if (topic[strlen(topic) - 1] == 'B') {
        pixelBrightness(0, payload, mLength);
      } else {
        pixelRgb(nr, payload, mLength);
        pixelType = NEOPIXEL_KEYPAD;
      }
      //pixelRgb(nr, payload, mLength);
    #endif
  } else if (slash == '/') {
    set(request, nr, receivedChar);
  } else if (String(topic).endsWith("/OSS")) {
    callbackOSS(payload, mLength);
  }
  yield();
  mqttSet((char*)"ack", cmessage);
}

// CR+LF, eventuell anpassen unter Linux ... == 0x0D
#define serialPosEnd 0x0A
#define serialKeypadEnd '#'
#define serialPosMax 30
char serialIn[serialPosMax + 1];
byte serialPos = 0;


void serialInSet(char in) {
  timerKeypadreject.activate();
  if (serialPos < serialPosMax) {
    serialIn[serialPos++] = in;
    serialIn[serialPos] = 0;
  } else {
    serialIn[serialPos - 1] = serialPosEnd;
  }
}

String base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
  static const String base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];
  ret.reserve(60);
  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; (i < 4); i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];
    /*missing in PERL SHA256_base64()
    while((i++ < 3))
      ret += '=';
    */
  }
  return ret;
}

void readInput() {
  if (Serial.available() > 0) {
    serialInSet(Serial.read());
  }
  if (serialIn[serialPos - 1] == serialPosEnd) {
    boolean erg = false;
    //keypad-Eingabe mit '#' abgeschlossen
    if (serialIn[serialPos - 2] == serialKeypadEnd) {
      serialIn[serialPos - 2] = '\0';
      serialPos = serialPos - 2;
      if (serialPos > 1) {
        //DEBUG3_PRINT("Serial out: ");
        //DEBUG3_PRINTLN(serialIn);
        SHA256 crypto;
        byte sha256hash[SHA256_SIZE];
        crypto.doUpdate(serialIn);  //, serialPos-3);
        crypto.doFinal(sha256hash);

        /* hash now contains our 32 byte hash */
        for (byte i = 0; i < SHA256_SIZE; i++) {
          //if (sha256hash[i]<0x10) { Serial.print('0'); }
          //Serial.print(sha256hash[i], HEX);
        }
        //Serial.println("");
        int qos = 11;
        while (!erg && (qos-- > 0)) {
          // PIN in spezielles Topic einfügen. Dieses Topic sollte nur FHEM lesen können
          erg = client.publish(para.mKeypad, base64_encode(sha256hash, SHA256_SIZE).c_str(), false);
          //DEBUG_PRINT(erg);
        }
        //DEBUG_PRINT(" ");
        //DEBUG_PRINTLN(base64_encode(sha256hash, SHA256_SIZE));
      } else {
        erg = client.publish(para.mKeypad, serialIn, false);
      }
      erg = true;
    }
    if (!erg) {
      char request = serialIn[0];
      byte nr = serialIn[1] - '0';
      char payload = serialIn[2];

      if (request == 'c') {
        //Einstellen();
        mqttReconnect();
      } else if (request == 'b') {
        bleScanActive = !bleScanActive;
        DEBUG_PRINT("BLE-Scan: ");
        DEBUG_PRINTLN(bleScanActive);
#ifdef BLE
        blePrintIRK();
#endif
      } else if (request == 'f') {
#ifndef ESP32
        FSInfo fs_info;
        Serial.println("Please wait 30 secs for SPIFFS to be formatted");
        LittleFS.format();
        Serial.println("Spiffs formatted");
        //See more at: http://www.esp8266.com/viewtopic.php?f=29&t=8194#sthash.mj02URAZ.dpuf
        LittleFS.info(fs_info);
        DEBUG_PRINT("totalBytes ");
        DEBUG_PRINT(fs_info.totalBytes);
#else
        LittleFS.format();
#endif
      } else if (request == 'i') {
        i2cScan();
      } else if (request == 'j') {
        setup1wire(true);
      } else if (request == 'l') {
        listSpiffs();
      } else if (request == 'w') {
        Serial.println("");
        Serial.println("Mit Wlan verbunden");
        Serial.print("IP Adresse: ");
        Serial.println(WiFi.localIP());
        //Serial.println("Zeit: " + PrintDate(now()) + " " + PrintTime(now()));
        //printUser();
        Serial.println(getSsid());
        Serial.println(getConfig());
        Serial.println(getIndex());
      } else if (nr < PIN_MAX && (request == 'A' || request == 'C' || request == 'D' || request == 'S' || request == 'O' || request == 'P')) {
        set(request, nr, payload);
      } else if (request == 'I') {
        #ifdef USE_IR      
          send_ir(nr, (byte*)serialIn + 2, serialPos - 2);
        #endif          
      } else if (request == 'M') {
        #ifdef USE_NEOPIXEL
          if (serialIn[1] == 'A') {
            pixelRgb(0, (byte*)serialIn + 2, serialPos - 2, NEOPIXEL_COUNT - 1);
          } else {
            pixelRgb(nr, (byte*)serialIn + 2, serialPos - 2);
          }
        #endif
      }
    }
    serialPos = 0;
    timerKeypadreject.deactivate();
  }
}

void setupPinmode() {
  byte i2csda = 0;
  byte i2cscl = 0;
  para.i2c = 0;
  para.onewire = 0;
  pwmPin = -1;
  ledPin = -1;
  for (byte i = 0; i < PIN_MAX; ++i) {
    switch (para.pin[i]) {
      case PIN_SENSOR:
      case PIN_ALARM:
        pinMode(Pin[i], INPUT_PULLUP);
        pinState[i] = para.GpioOn[i] == 0;  //ok
        mqttSet(PinName[i], tochararray(cstr, (char*)"INPUT_PULLUP ", tochararray(cpart, pinState[i])));
        break;
      case PIN_ACTOR:
      case PIN_SWITCH:
        pinMode(Pin[i], OUTPUT);
        // default: Switch off, LED off
        pinState[i] = !para.GpioOn[i];
        digitalWrite(Pin[i], pinState[i]);
        mqttSet(PinName[i], tochararray(cstr, (char*)"OUTPUT ", tochararray(cpart, pinState[i])));
        break;
      case PIN_LED:
        pinMode(Pin[i], OUTPUT);
        // default: Switch off, LED off
        pinState[i] = !para.GpioOn[i];
        digitalWrite(Pin[i], pinState[i]);
        ledPin = Pin[i];
        ledPinOn = para.GpioOn[i];
        mqttSet(PinName[i], tochararray(cstr, (char*)"LED ", tochararray(cpart, pinState[i])));
        break;
      case PIN_1WIRE:
        ONE_WIRE_BUS = Pin[i];
        para.onewire = 1;
        mqttSet(PinName[i], tochararray(cstr, (char*)"1Wire ", tochararray(cpart, ONE_WIRE_BUS)));
        break;
      case PIN_SCL:
        sclPin = Pin[i];
        i2cscl++;
        mqttSet(PinName[i], tochararray(cstr, (char*)"WireSCL ", tochararray(cpart, sclPin)));
        break;
      case PIN_SDA:
        sdaPin = Pin[i];
        i2csda++;
        mqttSet(PinName[i], tochararray(cstr, (char*)"WireSDA ", tochararray(cpart, sdaPin)));
        break;
      case PIN_PWM:
        pwmPin = Pin[i];
        pwmPinOn = para.GpioOn[i];
        pinMode(Pin[i], OUTPUT);
        pinState[i] = !para.GpioOn[i];
        digitalWrite(Pin[i], pinState[i]);
#ifdef ESP32
        ledcSetup(0, 1000, 8);  // channel, freq, resolution
        ledcAttachPin(Pin[i], 0);
#endif
        mqttSet(PinName[i], tochararray(cstr, (char*)"PWM ", tochararray(cpart, pinState[i])));
        break;
#ifdef USE_NEOPIXEL
      case PIN_PIXEL:
        pixelPin = Pin[i];
        mqttSet(PinName[i], tochararray(cstr, (char*)"Pixel ", tochararray(cpart, pixelPin)));
        break;
#endif
      default:
        mqttSet(PinName[i], tochararray(cstr, (char*)"UNDEF ", tochararray(cpart, para.pin[i])));
        break;
    }
  }
  if (i2csda == 1 && i2cscl == 1) {
    para.i2c = 1;
  }
  pinMode(A0, INPUT);
}

void testPara() {
  if (para.checksum != 123456) {
    para.checksum = 999999;
  }
  if (para.timerMsec[0] < 100) para.timerMsec[0] = 1000;
  if (para.timerMsec[1] < 1000) para.timerMsec[1] = 60000;
  if (para.timerMsec[2] < 1000) para.timerMsec[2] = 20000;
}

void setupI2c(boolean rescan) {
  if (para.i2c) {
    Wire.begin(sdaPin, sclPin);  //ESP32 ...,100000 ?
    i2cScan();
    if (!bmpActive) {
      // in bmp.begin() keine Pinzuordnung möglich
      // bmpActive = bmp.begin(sdaPin, sclPin);
      bmpActive = bmp.begin();
      bmpTyp = bmp.sensorID();
      //bmpADDR = bmp.sensorADDR();
      bmeActive = (bmpTyp == 0x60);
      if (bmpActive) {
        DEBUG1_PRINT("Addr 0x");
        //DEBUG1_PRINT(bmpADDR, HEX);
        DEBUG1_PRINT(", Typ 0x");
        DEBUG1_PRINTLN(bmpTyp, HEX);
        mqttSet((char*)"BMP-Typ", tochararray(cstr, bmpTyp));  //"-0x"+String(bmp.sensorID(), HEX));
        DEBUG1_PRINT("BMP init success! ");
        bmp.setSampling(Adafruit_BME280::MODE_NORMAL,  // Operating Mode.
                        Adafruit_BME280::SAMPLING_X1,  // Temp. oversampling
                        Adafruit_BME280::SAMPLING_X1,  // Pressure oversampling
                        Adafruit_BME280::SAMPLING_X1,  // Pressure oversampling
                        Adafruit_BME280::FILTER_OFF,   // Filtering.
                        Adafruit_BME280::STANDBY_MS_1000);
      } else {
        DEBUG1_PRINT("BMP failed! ");
      }
    }
  #ifdef USE_BH1750
    if (!bh1750Active) {
      bh1750Active = bh1750.begin();
      if (bh1750Active) {
      DEBUG1_PRINTLN("Lightsensor init success!");
      }
    }
  #endif
  #ifdef USE_BH1745
    if (!bh1745Active) {
      bh1745Active = bh1745setup();
      delay(30);
      if (bh1745Active) {
        DEBUG1_PRINTLN("Colorsensor init success!");
      }
    }
  #endif
  #ifdef USE_ADS1X15
    for (byte addr = 0; addr < 3; addr++)
    if (!adsActive[addr]) {
      if (addr == 0) adsActive[addr] = ads[addr].begin(72);
      if (addr == 1) adsActive[addr] = ads[addr].begin(73);
      if (addr == 2) adsActive[addr] = ads[addr].begin(75);
      yield();
      DEBUG1_PRINTF("A/D init %d success: %d \n", addr, adsActive[addr]);
    }
  #endif
    if (!pcf8574Active) {
      pcf8574Active = (pcf8574.begin() == 0);
      DEBUG_PRINT("Keypad pcf8574 ");
      DEBUG_PRINTLN(pcf8574Active);
    }
  }
  if (!para.i2c) {
    DEBUG1_PRINTLN("I2C failed!");
  } else if (!bmpActive and !bh1750Active) {
    DEBUG1_PRINTLN("BMP/BH1750 init failed!");
  }
  if (rescan && para.i2c) {
    DEBUG1_PRINTLN("I2C Scanner");
    i2cScan();
  }
}

void setup1wire(boolean rescan) {
  #ifdef USE_DS_SENSORS  
    if (para.onewire) {
      if (!dsActive || rescan) {
        dsActive = dsSetup(rescan);
      }
    }
    if (!dsActive){
      DEBUG1_PRINTLN("no 1wire");
    }    
  #endif
}

// default tone() / noTone() erhitzt meine passiven Buzzer,
// unbedingt nach Nutzung auf !pwmPinOn.
// default 0, benötigt 1 =>  pwmPinOn auf 0 einstellen
/* ESP32 hat andere Bibliotheken
#       ifdef ESP32
          ledcWrite(0, 128);
#       endif
*/

#ifdef USE_NEOPIXEL
void setupPixel() {
  if (pixelPin < 0xFF) {
    strip.setPin(pixelPin);
    strip.begin();                     // Initialize NeoPixel strip object (REQUIRED)
    strip.setPixelColor(0, 0x000000);  //  Set pixel's color (in RAM)
    strip.show();                      // Initialize all pixels to 'off'
    pixelChanged = 1;
  }
}

void pixelLoop() {
  if (pixelPin < 0xFF && pixelChanged) {
    strip.show();  //  Update strip to match
    pixelChanged = 0;
  }
}

void pixelSet(uint16_t nr, byte type) {
  uint8_t r = 0, g = 0, b = 0;
  switch (type) {
    case PIXEL_ON:
      g = 0x04;
      break;
    case PIXEL_ALARM:
      r = 0x04;
      break;
    case PIXEL_SET:
      b = 0x04;
      break;
    case PIXEL_NA:
      r = 0x02;
      b = 0x02;
      break;
  }
  Serial.print(nr);
  Serial.print(" T ");
  Serial.print(type);
  Serial.println("RGB");
  strip.setPixelColor(nr, r, g, b);  //  Set pixel's color (in RAM)
}
#endif

void pixelRgb(byte nr, byte* payload, unsigned int mLength, byte to_nr) {
#ifdef USE_NEOPIXEL
  if (mLength == 1) {
    pixelSet(nr, payload[0] - '0');
  } else {
    uint32_t c = 0;  //payload[0]-'0';
    for (byte i = 0; i < 6; i++) {
      // get current character then increment
      char rgb = payload[i];
      // transform hex character to the 4bit equivalent number, using the ascii table indexes
      if (rgb >= '0' && rgb <= '9') rgb = rgb - '0';
      else if (rgb >= 'a' && rgb <= 'f') rgb = rgb - 'a' + 10;
      else if (rgb >= 'A' && rgb <= 'F') rgb = rgb - 'A' + 10;
      // shift 4 to make space for new digit, and add the 4 bits of the new digit
      c = (c << 4) | (rgb & 0xF);
    }
    Serial.print(nr);
    Serial.print(" C ");
    Serial.print(c, HEX);
    Serial.println("RGB");
    for (byte i = nr; i == nr || i <= to_nr; i++) {
      strip.setPixelColor(i, c);  //  Set pixel's color (in RAM)
    }
  }
  pixelChanged = 1;
#endif
}
void pixelRgb(byte nr, byte* payload, unsigned int mLength) {
  pixelRgb(nr, payload, mLength, 0);
}

void pixelBrightness(byte nr, byte* payload, unsigned int mLength) {
#ifdef USE_NEOPIXEL
  uint8_t c = 0;  //payload[0]-'0';
  for (byte i = 0; i < 2; i++) {
    // get current character then increment
    char rgb = payload[i];
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (rgb >= '0' && rgb <= '9') rgb = rgb - '0';
    else if (rgb >= 'a' && rgb <= 'f') rgb = rgb - 'a' + 10;
    else if (rgb >= 'A' && rgb <= 'F') rgb = rgb - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    c = (c << 4) | (rgb & 0xF);
  }
  bh1745_brightness = c;
  Serial.print(nr);
  Serial.print(" B ");
  Serial.print(c, HEX);
  Serial.println("RGB");
#endif
}

#ifdef USE_IR
void loop_ir(void) {
  if (irrecv.decode(&results)) {
    uint64_t input = results.value;
    char* type = (char*)"IR";
    char* result = (char*)"                  ";
    char c;
    int i = 17;
    do {
      c = input % 16;
      input /= 16;
      if (c < 10)
        c += '0';
      else
        c += 'A' - 10;
      result[i--] = c;
    } while (input);

    c = (results.decode_type & 0xF);
    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result[i--] = c;
    c = (results.decode_type >> 4 & 0xF);
    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result[i--] = c;
    mqttSet((char*)"code", result + i + 1);
    // print() & println() can't handle printing long longs. (uint64_t)
    Serial.print(type);
    Serial.print(" ");
    //serialPrintUint64(results.value, HEX);
    //Serial.print(resultToHumanReadableBasic(&results));
    Serial.println(result + i + 1);

    irrecv.resume();  // Receive the next value
  }
}
void send_ir(char type, byte* input, unsigned int mLength) {
  uint64_t c = 0;
  int t = 0;

  for (byte i = 0; i < 2; i++) {
    // get current character then increment
    char rgb = input[i];
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (rgb >= '0' && rgb <= '9') rgb = rgb - '0';
    else if (rgb >= 'a' && rgb <= 'f') rgb = rgb - 'a' + 10;
    else if (rgb >= 'A' && rgb <= 'F') rgb = rgb - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    t = (t << 4) | (rgb & 0xF);
  }
  for (byte i = 2; i < mLength; i++) {
    // get current character then increment
    char rgb = input[i];
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (rgb >= '0' && rgb <= '9') rgb = rgb - '0';
    else if (rgb >= 'a' && rgb <= 'f') rgb = rgb - 'a' + 10;
    else if (rgb >= 'A' && rgb <= 'F') rgb = rgb - 'A' + 10;
    // shift 4 to make space for new digit, and add the 4 bits of the new digit
    c = (c << 4) | (rgb & 0xF);
  }
  decode_type_t td = (decode_type_t)t;
  Serial.print("IR~");
  Serial.print(td, HEX);
  Serial.print("~");
  serialPrintUint64(c, HEX);
  Serial.println("~IR");

  irrecv.disableIRIn();  // Start the receiver
  irsend.send(td, c, irsend.defaultBits(td), irsend.minRepeats(td));
  irrecv.enableIRIn();  // Start the receiver
  /*
    switch(t){
      case 0x02 :   Serial.print("RC6"); irsend.sendRC6(c); break;
      case 0x03 :   Serial.print("NEC"); irsend.sendNEC(c); break;
      default   :       
    }*/
}
#endif

void keyclick(boolean clicking, int freq, int period) {
  if (period != 0) {
    digitalWrite(ledPin, ledPinOn);
    if (clicking && pwmPin >= 0) {
      digitalWrite(pwmPin, pwmPinOn);
      tone(pwmPin, freq);
    }
    if (period > 0)
      delay(period);
  }
  if (period >= 0) {
    if (clicking && pwmPin >= 0) {
      noTone(pwmPin);
      digitalWrite(pwmPin, !pwmPinOn);
    }
    digitalWrite(ledPin, !ledPinOn);
  }
}

#define keymapRows 4
#define keymapCols 4
#define keymapClick true
boolean keymapPause = true;
uint8_t pcf8574Last[keymapCols] = { 0 };
const char keymap[keymapRows][keymapCols + 1] = {
  "123A",
  "456B",
  "789C",
  "*0#D"
};
void keypadloop() {
  uint8_t i, j = 0;
  uint8_t test = 0;
  uint8_t keymapX = 0;
  if (pcf8574Active) {
    test = pcf8574.read8();
    //if (test != 0x0F){
    if (test != 0xF0) {
      //DEBUG_PRINT("keypad1 ");
      //DEBUG_PRINTLN(test, BIN);
      for (i = 0; i < keymapCols; i++) {
        pcf8574.write8(~(0x08 >> i));  // links nach rechts
        //pcf8574.write8(0xF0 | (0x08 >> i)); // links nach rechts
        delay(3);
        test = ~(pcf8574.read8());
        //DEBUG_PRINT("keypad2 ");
        //DEBUG_PRINTLN(test, BIN);
        if (keymapPause) {
          pcf8574Last[i] = 0;
          keymapPause = false;
        }
        if (test != pcf8574Last[i]) {
          pcf8574Last[i] = test;
          if (test > 0x0F) {
            for (j = 0; j < keymapRows; j++) {
              keymapX = test & (0x80 >> j);
              if (keymapX) {
                keyclick(keymapClick, 220, 100);
                serialInSet(keymap[j][i]);
                if (keymap[j][i] == serialKeypadEnd) {
                  serialInSet(0x0A);
                }
                //DEBUG2_PRINT("keypad ");
                //DEBUG2_PRINTLN(keymap[j][i]);
              }
            }
          }
        }
      }
      pcf8574.write8(0xF0);  // warten, ob irgendeine Taste gedrückt wird
    } else {
      keymapPause = true;
    }
  }
}

void keypadreject() {
  serialPos = 0;
  keyclick(keymapClick, 220, 100);
}

void ads1x15_loop(void)
{
  #ifdef USE_ADS1X15
    int16_t results;

    /* Be sure to update this value based on the IC and the gain settings! */
    //float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
    float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
    for (uint8_t addr = 0; addr < 3; addr++){
      if (adsActive[addr]){
        for (uint8_t port = 0; port < 4; port++){
          uint8_t channel = addr * 4 + port;
          results = ads[addr].readADC_SingleEnded(port);
          yield();
          mqttSet(tochararray(cstr, (char*)"AD", channel), tochararray(cmessage, results, 4, 1));
          Serial.print("Differential: "); Serial.print(cstr); Serial.print(cmessage); Serial.print(results * multiplier); Serial.println("mV)");
        }
      }
    }
  #endif
}

void setup() {
  Serial.begin(115200);
  DEBUG1_PRINTLN(ARDUINO_BOARD);
  DEBUG1_PRINTLN(ARDUINO_VARIANT);
  DEBUG1_PRINTLN(F_CPU);
  DEBUG1_PRINTLN(board);
  // getSystem();
  getPara();
  // byte i;
  DEBUG1_PRINTLN("getPara");

  setupPinmode();
  DEBUG1_PRINTLN("setupPinmode");
  delay(100);
  setupI2c(false);
  delay(100);
  DEBUG1_PRINTLN("setupI2c");
  setup1wire(false);
  DEBUG1_PRINTLN("setup1wire");
#ifdef USE_NEOPIXEL
  setupPixel();
  //PixelSet(9, PIXEL_ALARM);
#endif
#ifdef USE_IR
  setup_ir();
#endif
  timerAlarmloop.begin(1000, Alarmloop);
  DEBUG1_PRINTLN("timerAlarmloop");
  timerSensors.begin(para.timerMsec[1], getData);
  DEBUG1_PRINTLN("timerSensors");
  timerKeypadloop.begin(100, keypadloop);
  timerKeypadreject.begin(10000, keypadreject, false, true);
#ifdef BLE
  setupBLE();
#endif
#ifndef ESP32
  stationDisconnectedHandler = WiFi.onStationModeDisconnected(onDisconnect);
  stationGotIpHandler = WiFi.onStationModeGotIP(onGotIP);
#endif
  wifiSetup(para.pVersion > 0);  // && ( para.checksum == 123456 || para.checksum = 999999)
  DEBUG1_PRINTLN("wifiSetup");
  httpSetup();
  DEBUG1_PRINTLN("httpSetup");

  if (wifiStation) {
#ifdef USE_NEOPIXEL
    //PixelSet(9, PIXEL_SET);
#endif
    yield();
    mqttSetup();
    yield();
    timerMqtt.begin(para.timerMsec[0], mqttSend);
    yield();
    timerReconnect.begin(para.timerMsec[2], mqttReconnect);
    yield();
    timerNtp.begin(86400000, getNTP);
  }
  yield();
  timerAlarmstate.begin(333, setAlarmLED, false);
#ifdef USE_NEOPIXEL
  //PixelSet(9, PIXEL_ON);
#endif
}

void loop() {
  timerAlarmloop.update();
  yield();
  timerAlarmstate.update();
  yield();
  timerSensors.update();
  yield();
  timerKeypadloop.update();
  yield();
  timerKeypadreject.update();
  yield();
  timerRestartDelay.update();
  yield();
  if (wifiStation) {
    timerMqtt.update();
    yield();
    timerReconnect.update();
    yield();
    timerNtp.update();
  }
  yield();
  mqttLoop();
  yield();
#ifdef USE_NEOPIXEL
  pixelLoop();
  yield();
#endif
#ifdef USE_IR
  loop_ir();
  yield();
#endif
#ifdef USE_BH1745
  if (bh1745Active) {
    bh1745loop();
    yield();
  }
#endif
  http.handleClient();
  yield();
  readInput();
#ifdef BLE
  bleLoop();
#endif
}
