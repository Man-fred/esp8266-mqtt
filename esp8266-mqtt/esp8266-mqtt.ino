char mVersionNr[] = "V02-00-00.esp8266-mqtt.ino.";
#ifndef DBG_OUTPUT_PORT
  #define DBG_OUTPUT_PORT Serial
#endif
#define DEBUG 3
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
#ifdef ARDUINO_ESP8266_NODEMCU
# include <ESP8266WiFi.h>
// enables OTA updates
# include <ESP8266httpUpdate.h>
// enables webconfig
# include <ESP8266WebServer.h>
# include <ESP8266mDNS.h>
# define PIN_MAX 9
  const byte board = 1;
  const byte Pin[PIN_MAX] = {D7, D6, D5, D8, D4, D1, D2, D0, D3}; // reed3 oder S3
  char* PinName[] = {(char*)"D.7", "D.6", "D.5", "D.8", "D.4", "D.1", "D.2", "D.0", "D.3"};
  String mVersionBoard = "nodemcu";

#elif ARDUINO_ESP8266_WEMOS_D1MINI
# include <ESP8266WiFi.h>
// enables OTA updates
# include <ESP8266httpUpdate.h>
// enables webconfig
# include <ESP8266WebServer.h>
# include <ESP8266mDNS.h>
# define PIN_MAX 9
  const byte board = 2;
  const byte Pin[] = {D7, D6, D5, D0, D3, D1, D2, D4, D8}; 
  char *PinName[] = {"D.7", "D.6", "D.5", "D.0", "D.3", "D.1", "D.2", "D.4", (char*)"D.8"};
  char mVersionBoard[] = "d1_mini";

#elif ARDUINO_ESP32_NODEMCU_32S
# define PIN_MAX 9
# define SYSTEM_EVENT_STA_LOST_IP 8
# define SYSTEM_EVENT_GOT_IP6 19
# include <WiFi.h>
// enables OTA updates
# include <HTTPUpdate.h>
# define ESPhttpUpdate httpUpdate
// enables webconfig
# include <WebServer.h>
# include <ESPmDNS.h>
# include <SPIFFS.h>
  const byte board = 3;
  const byte Pin[] = {GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8}; // D5: reed3 in oder S3 out
  char *PinName[] = {"D.16", "D.17", "D.18", "D.19", "D.6", "D.22", "D.21", "D.2", "D.8"};
  String mVersionBoard = "nodemcu-32s";

#elif ESP32
# define PIN_MAX 9
# define WL_MAC_ADDR_LENGTH 8
//# define SYSTEM_EVENT_STA_LOST_IP 8
//# define SYSTEM_EVENT_GOT_IP6 19
# include <WiFi.h>
# include <WiFiScan.h>
// enables OTA updates
# include <HTTPUpdate.h>
# include <HTTPClient.h>

// enables webconfig
# include <WebServer.h>
# include <ESPmDNS.h>
# include <SPIFFS.h>
# include <rom/rtc.h>
  const byte board = 3;
  const byte Pin[] = {GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8}; // D5: reed3 in oder S3 out
  char *PinName[] = {"D.16", "D.17", "D.18", "D.19", "D.6", "D.22", "D.21", "D.2", "D.8"};
  String mVersionBoard = "esp32";

#else
# define PIN_MAX 9
# define SYSTEM_EVENT_STA_LOST_IP 8
# define SYSTEM_EVENT_GOT_IP6 19
# include <WiFi.h>
// enables OTA updates
# include <HTTPUpdate.h>
# define ESPhttpUpdate httpUpdate
// enables webconfig
# include <WebServer.h>
# include <ESPmDNS.h>
# include <SPIFFS.h>
  const byte board = 3;
  const byte Pin[] = {GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_6, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_2, GPIO_NUM_8}; // D5: reed3 in oder S3 out
  char *PinName[] = {"D.16", "D.17", "D.18", "D.19", "D.6", "D.22", "D.21", "D.2", "D.8"};
  String mVersionBoard = "unknown";
#endif
byte ONE_WIRE_BUS = Pin[4];
byte sclPin = Pin[5];
byte sdaPin = Pin[6];
byte ledPin = Pin[7]; //BUILTIN_LED; 

// enables storing webpages in EEPROM, not in sketch
#include <FS.h>
// enables storing config-data in EEPROM
#include <EEPROM.h>
// enables NTP / RTC
#include <WiFiUdp.h>

//#define MQTT_MAX_PACKET_SIZE 256 // default 128
#include <PubSubClient.h>
#include <Wire.h>
//#include <BMP280.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DS2450.h>

/*  V01-00 : Arduino 1.6.7
 *  V01-01 : Arduino 1.6.10, DallasTemperature, OneWire updated
 *  V01-04 : Arduino 1.6.10, webserver for configdate, saved in eeprom, detection of NodeMCU and WeMos D1 mini
 *  V01-05 : Arduino 1.8.2,  
 *  V01-06 : Arduino 1.8.9,  
 */
#define cstrLen 50
char cstr[cstrLen];
char cmessage[cstrLen];
char cpart[cstrLen];
const byte hex[17] = "0123456789ABCDEF";

char serialIn[5];
byte serialPos = 0;

// Parameters for WiFi and MQTT
#ifdef ESP32
  //WiFiEventHandler stationGotIpHandler, stationDisconnectedHandler;
#else
  WiFiEventHandler stationGotIpHandler, stationDisconnectedHandler;
#endif

struct Parameter {
  unsigned char pVersion;
  char ssid[20];
  char password[20];
  
  char mqtt_server[20];
  int mqtt_port;
  char mqtt_user[20];
  char mqtt_pass[20];
  
  uint8_t  mMac[6] = {0, 0, 0, 0, 0, 0};
  char mClient[20];
  
  char mPre[10];
  char mSub[10];
  char mLwt[10];
  unsigned int pVersion2;
  
  int timerMsec[3];
  byte pin[PIN_MAX]; // 0: inaktiv, 1: Sensor, 2: Schalter, 3: Alarm, 4: PWM
  byte GpioOn[PIN_MAX];
  byte GpioLedOn;
  int analog;
  bool i2c;
  bool onewire;
  unsigned int checksum;
};

/* Set the tm_t fields for the local time. */
struct NtpTime {
  unsigned long epoch, isdst, sec, min, hour, wday, leap, year, mon, yday, mday, startup, delta;
};

struct NtpTime ntpTime;

struct Parameter para;

//String mChipId = "";
//String mClient = "";
unsigned int mFlashSize = 0;
boolean inSetup = true;

//========= Variables for sketch-part "Http" ===============
WiFiClient espClient;
#ifdef ESP32
  WebServer http(80);
#else
  ESP8266WebServer http(80);
#endif
//holds the current upload
File fsUploadFile;
//========= Variables for sketch-part "Http" ===============

//========= Variables for sketch-part "Ntp" ===============
// A UDP instance to let us send and receive packets over UDP1
WiFiUDP udp;
unsigned int localPort = 2390;      // local port to listen for UDP packets
/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
//========= Variables for sketch-part "Ntp" ===============

PubSubClient client(espClient);
boolean wifiStation = false;
boolean mqttConnected = false;

Adafruit_BME280 bmp;
boolean bmpActive = false;
boolean bmeActive = false;
char bmpTyp;
double bmpT = 0;
double bmpP = 0;
double bmeH = 0;
unsigned long bmpTime;
//#define bmpP0 1013.25

BH1750FVI bh1750;
boolean bh1750Active = false;
uint16_t bh1750Lux = 0;
unsigned long bh1750Time;

OneWire ds(ONE_WIRE_BUS); /* Ini oneWire instance */
DallasTemperature ds18b20(&ds);/* Dallas Temperature Library für Nutzung der oneWire */
DS2450 ds2450(&ds);

#define MAX_DS_SENSORS 3
boolean dsActive = false;
int ds1820Sensors = 0;
int ds2450Sensors = 0;
byte dsAddr[MAX_DS_SENSORS][8];
float dsTemp[MAX_DS_SENSORS];
float dsAD[4];
unsigned long dsTime[MAX_DS_SENSORS];

int pinState[PIN_MAX];
int reedTimer[PIN_MAX];

int reedAlarmstate = 0; // 0: not armed, 1-7: armed
int reedSensor = 0;      // 0: no action, 1: Gpio1, 2: Gpio2, 4: Gpio3, 8: Gpio4
int reedActor = 0;      // 0: no action, 1: internal, 2: light, 4: external sirene, 8: reserve
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
//boolean SOn[] = {false, false, false, false};

int analogWert = 0;
int analogState = 0; // Analog < 512 -> 0, > 511 -> 1

#if DEBUG > 2
  #define DEBUG3_PRINT(x, ...)   DBG_OUTPUT_PORT.print (x, ## __VA_ARGS__)
  #define DEBUG3_PRINTLN(x, ...) DBG_OUTPUT_PORT.println (x, ## __VA_ARGS__)
  #define DEBUG3_PRINTF(x, ...)  DBG_OUTPUT_PORT.printf (x, ## __VA_ARGS__)
#else
  #define DEBUG3_PRINT(x, ...) 
  #define DEBUG3_PRINTLN(x, ...) 
  #define DEBUG3_PRINTF(x, ...)
#endif
#if DEBUG > 1
  #define DEBUG2_PRINT(x, ...)   DBG_OUTPUT_PORT.print (x, ## __VA_ARGS__)
  #define DEBUG2_PRINTLN(x, ...) DBG_OUTPUT_PORT.println (x, ## __VA_ARGS__)
  #define DEBUG2_PRINTF(x, ...)  DBG_OUTPUT_PORT.printf (x, ## __VA_ARGS__)
#else
  #define DEBUG2_PRINT(x, ...) 
  #define DEBUG2_PRINTLN(x, ...) 
  #define DEBUG2_PRINTF(x, ...)
#endif
#if DEBUG > 0
  #define DEBUG1_PRINT(x, ...)   DBG_OUTPUT_PORT.print (x, ## __VA_ARGS__)
  #define DEBUG1_PRINTLN(x, ...) DBG_OUTPUT_PORT.println (x, ## __VA_ARGS__)
  #define DEBUG1_PRINTF(x, ...)  DBG_OUTPUT_PORT.printf (x, ## __VA_ARGS__)
#else
  #define DEBUG1_PRINT(x, ...) 
  #define DEBUG1_PRINTLN(x, ...) 
  #define DEBUG1_PRINTF(x, ...)
#endif
#define DEBUG_PRINT(x, ...)   DBG_OUTPUT_PORT.print (x, ## __VA_ARGS__)
#define DEBUG_PRINTLN(x, ...) DBG_OUTPUT_PORT.println (x, ## __VA_ARGS__)
#define DEBUG_PRINTF(x, ...)  DBG_OUTPUT_PORT.printf (x, ## __VA_ARGS__)

#define MAX_MESSAGES 40
#define MQTT_MAX_TOPIC_SIZE 50
// MQTT_MAX_PACKET_SIZE 128
char mPayloadKey[MAX_MESSAGES][MQTT_MAX_TOPIC_SIZE];
char mPayloadValue[MAX_MESSAGES][MQTT_MAX_PACKET_SIZE];
boolean mPayloadRetain[MAX_MESSAGES];
int mPayloadQos[MAX_MESSAGES];
int mPayloadSet = 0;
int mPayloadPublish = 0;

char* tochararray(char* cvalue, int value){
  itoa(value, cvalue, 10);
  return cvalue;
}
char* tochararray(char* cvalue, unsigned int value){
  utoa(value, cvalue, 10);
  return cvalue;
}

char* tochararray(char* cvalue, float value, int len, int nachkomma){
  dtostrf(value, len, nachkomma, cvalue);
  return cvalue;
}

char* tochararray(char* cvalue, char* value1, char value2){
  strcpy(cvalue, value1);
  byte len = strlen(value1);
  cvalue[len] = hex[(value2 & 0xF0) >> 4];
  cvalue[len+1] = hex[value2 & 0x0F];
  cvalue[len+2] = '\0';
  return cvalue;
}
char* tochararray(char* cvalue, char* value1, char* value2){
  strcpy(cvalue, value1);
  if (value2[0] != '\0')
    strcat(cvalue, value2);
  cvalue[strlen(value1)+strlen(value2)] = '\0';
  return cvalue;
}
char* tochararray(char* cvalue, char* value1){
  strcpy(cvalue, value1);
  return cvalue;
}
char* tochararray(char* cvalue, String value1){
  value1.toCharArray(cvalue, cstrLen);
  return cvalue;
}
char* tochararray(char* cvalue, String value1, String value2){
  char cpart[cstrLen];
  value1.toCharArray(cvalue, cstrLen);
  value2.toCharArray(cpart, cstrLen);
  strcat(cvalue, cpart);
  return cvalue;
}

class MyTimer {
    // Class Member Variables
    // These are initialized at startup
    int aPeriod;
    boolean aActive;
    void (*aCallback) ();
    unsigned long previousMillis;   // will store last callback-time
    unsigned long currentMillis;

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    // MyTimer();
    
    void begin(int period, void(*func)(), boolean activated = true)
    {
      aActive = activated;
      aPeriod = period;
      aCallback = func;
      // max. unsigned int to start the timer immediately
      previousMillis = 4294967295; 
    }

    void activate() {
      aActive = true;
    }

    void deactivate() {
      aActive = false;
    }

    void update() {
      if (aActive) {
        currentMillis = millis();
        // check to see if it's time to start callback
        // Ueberlauf abfangen
        if ( (currentMillis < previousMillis) || (currentMillis - previousMillis >= aPeriod) )
        {
          previousMillis = currentMillis;   // Remember the time
          yield();
          aCallback();
        }
      }
    }
};

MyTimer timerMqtt; // mqtt delay
MyTimer timerSensors; // collect 1wire/i2c sensors delay
MyTimer timerReconnect; // reconnect wifi delay
MyTimer timerNtp; // ntp-loop 
MyTimer timerRestartDelay; // restart delay in websession
MyTimer timerAlarmloop; // alarm sensors delay
MyTimer timerAlarmstate; // blink LED for local alarm

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
void testPara() {
  byte i2csda = 0;
  byte i2cscl = 0;
  para.i2c = 0;
  para.onewire = 0;
  if (para.checksum != 123456) {
    para.checksum = 999999;
  }
  if (para.timerMsec[0] < 100) para.timerMsec[0] = 1000;
  if (para.timerMsec[1] < 1000) para.timerMsec[1] = 60000;
  if (para.timerMsec[2] < 1000) para.timerMsec[2] = 20000;
  for (byte i = 0; i < PIN_MAX; ++i) {
    switch (para.pin[i]) {
      case PIN_1WIRE : ONE_WIRE_BUS = Pin[i];
                       para.onewire = 1;
                       break;
      case PIN_SCL   : sclPin = Pin[i];
                       i2cscl++;
                       break;
      case PIN_SDA   : sdaPin = Pin[i];
                       i2csda++;
                       break;
      case PIN_LED   : ledPin = Pin[i];
                       para.GpioLedOn = para.GpioOn[i];
                       break;
    }
  }
  if (i2csda == 1 && i2cscl == 1) {
    para.i2c = 1;
  }
}

void getPara() {
  EEPROM.begin(4096);
  EEPROM.get(0, para);
  if (para.pVersion > 0 && (para.pVersion2 == (123456 + para.pVersion))) {
    DEBUG1_PRINTLN("Flash loaded");
    testPara();
  } else {
    // Default Parameter setzen //
    para.pVersion = 0;
    strncpy( para.ssid, "...", 20); para.ssid[20 - 1] = '\0';
    
    strncpy( para.mqtt_server, "192.168.178.60", 20); para.mqtt_server[20 - 1] = '\0';
    para.mqtt_port = 1883;
    /*strncpy( para.mClient, "ESP", 20); para.mClient[20 - 1] = '\0';
    for (byte i = 3; i < 6; ++i) {
      para.mClient[2+(i*2)] = hex[(para.mMac[i] & 0xF0) >> 4];
      para.mClient[3+(i*2)] = hex[para.mMac[i] & 0x0F];
    }*/
    strncpy( para.mClient,macToEsp(para.mMac),10);
    strncpy( para.mPre, "esp/", 10); para.mPre[10 - 1] = '\0';
    strncpy( para.mSub, "set/+", 10); para.mSub[10 - 1] = '\0';
    strncpy( para.mLwt, "lwt", 10); para.mLwt[10 - 1] = '\0';
    
    // timerMqtt: MQTT - Sendpipe 
    para.timerMsec[0] =  1000;
    para.timerMsec[1] = 60000;
    para.timerMsec[2] = 20000;
    for (byte i = 0; i < PIN_MAX; ++i) {
      para.pin[i] = PIN_NOTHING;
      para.GpioOn[i] = 0;
    }
    para.pin[4] = PIN_1WIRE;
    para.pin[5] = PIN_SCL;
    para.pin[6] = PIN_SDA;
    para.pin[7] = PIN_LED; //BUILTIN_LED; 
    para.GpioLedOn = 0;
    para.analog = 0;
    para.i2c = 0;
    para.onewire = 0;
    para.checksum = 111111;

    // 512k: 3c000 - 3ffff
    // 1024k 7c000
    // 2048/4096: 7c000 (max Flashsize 428k) oder fc000 (max Flashsize 940k)
    // 4096: zusaetzlich 1fc000 - 3fbfff (>= 2048 KB)
    EEPROM.put(0, para);
    EEPROM.commit();            // EEPROM Schreiben
    DEBUG1_PRINTLN("Flash written");
    // */
  }
  EEPROM.end();
}

void mqttSet(char* key, char* value, boolean retain = true, int qos = 0) {
  byte payload = strlen(para.mPre)+strlen(para.mClient)+1+strlen(key);
  if (payload < MQTT_MAX_TOPIC_SIZE) {
    strncpy(mPayloadKey[mPayloadSet], para.mPre, strlen(para.mPre));
    strncat(mPayloadKey[mPayloadSet], para.mClient, strlen(para.mClient));
    strncat(mPayloadKey[mPayloadSet], "/", 1);
    strncat(mPayloadKey[mPayloadSet], key, strlen(key));
    mPayloadKey[mPayloadSet][payload] = '\0';
    payload = strlen(value);
    strncpy(mPayloadValue[mPayloadSet], value, payload);
    mPayloadValue[mPayloadSet][payload] = '\0';
    mPayloadRetain[mPayloadSet] = retain;
    mPayloadQos[mPayloadSet] = qos;
    DEBUG3_PRINT("mqttSet  ");
    DEBUG3_PRINT(mPayloadSet);
    DEBUG3_PRINT(": ");
    DEBUG3_PRINT(mPayloadKey[mPayloadSet]);
    DEBUG3_PRINT(" <");
    DEBUG3_PRINT(value);
    DEBUG3_PRINTLN(">");
    mPayloadSet++;
    if (mPayloadSet > MAX_MESSAGES-1) {
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
    if (!erg && mPayloadQos[mPayloadPublish]-- > 0){
      mqttSet(mPayloadKey[mPayloadPublish], mPayloadValue[mPayloadPublish], mPayloadRetain[mPayloadSet], mPayloadQos[mPayloadPublish]);
    }
    DEBUG3_PRINT("mqttSend ");
    DEBUG3_PRINT(mPayloadPublish);
    DEBUG3_PRINT(": ");
    DEBUG3_PRINT(mPayloadKey[mPayloadPublish]);
    DEBUG3_PRINT(" <");
    DEBUG3_PRINT(mPayloadValue[mPayloadPublish]);
    DEBUG3_PRINTLN(">");
    memset(mPayloadKey[mPayloadPublish], 0, MQTT_MAX_TOPIC_SIZE);
    mPayloadPublish++;
    if (mPayloadPublish > MAX_MESSAGES-1) {
      mPayloadPublish = 0;
    }
  }
}

void bh1750loop() {
  bh1750Lux = bh1750.readLightLevel();
  bh1750Time = ntpTime.delta + millis() / 1000;
  mqttSet("lux", tochararray(cmessage, bh1750Lux));

  DEBUG1_PRINT("Light: ");
  DEBUG1_PRINT(bh1750Lux);
  DEBUG1_PRINTLN(" lx");
}

void bmp280loop() {
  bmp.takeForcedMeasurement();
  float t = bmp.readTemperature();
  if (abs(bmpT - t) > 0.009) {
      DEBUG1_PRINT("T = \t"); DEBUG1_PRINT(bmpT, 2); DEBUG1_PRINT(" deg ");DEBUG1_PRINT(t, 3);
      bmpT = t;
      if (ds1820Sensors > 0) {
        mqttSet("tempBMP", tochararray(cmessage, bmpT,3,1));
      } else {
        mqttSet("temp", tochararray(cmessage, bmpT, 3, 1));
      }
  }
  float p = bmp.readPressure();
  if (abs(bmpP - p) > 9) {
      DEBUG1_PRINT("P = \t"); DEBUG1_PRINT(bmpP/100, 0); DEBUG1_PRINT(" mBar ");DEBUG1_PRINT(p/100, 2);
      bmpP = p;
      mqttSet("pressure", tochararray(cmessage, float(bmpP/100), 5, 1));
  }
  if (bmeActive) {
    float h = bmp.readHumidity();
    if (abs(bmeH - h) > 0.09) {
        DEBUG1_PRINT("H = \t"); DEBUG1_PRINT(bmeH, 0); DEBUG1_PRINT(" % ");DEBUG1_PRINT(h, 2);
        bmeH = h;
        mqttSet("humidity", tochararray(cmessage, float(bmeH), 5, 1));
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
  byte i,j;
  byte present = 0;
  byte data[12];
  String addr = "";
  if (!ds1820Sensors){
    ds18b20.begin();
  }
  for (i = 0; i < MAX_DS_SENSORS; i++) {
    if (!ds.search(dsAddr[i]))
    {
      if (i == 0) {
        DEBUG1_PRINTLN("No OneWire addresses.");
      }
      ds.reset_search();
      return (i > 0);
    } else {
      DEBUG2_PRINT("R=");
      addr = "";
      for ( j = 0; j < 8; j++) {
        addr += String(dsAddr[i][j], HEX);
        addr += " ";
        DEBUG2_PRINT(dsAddr[i][j], HEX);
        DEBUG2_PRINT(" ");
      }
    
      if ( OneWire::crc8( dsAddr[i], 7) != dsAddr[i][7]) {
        DEBUG2_PRINT("CRC nicht gültig!\n");
      } else if ( dsAddr[i][0] == 0x10) {
        DEBUG2_PRINT("Sensor DS18S20\n");
        ds1820Sensors++;
        mqttSet(tochararray(cstr, "DS18S20",tochararray(cpart, i)), tochararray(cmessage, addr));
      }
      else if ( dsAddr[i][0] == 0x28) {
        DEBUG2_PRINT("Sensor DS18B20\n");
        ds1820Sensors++;
        mqttSet(tochararray(cstr, "DS18B20",tochararray(cpart, i)), tochararray(cmessage, addr));
      }
      else if ( dsAddr[i][0] == 0x20) {
        DEBUG2_PRINT("Sensor DS2450\n");
        if (!ds2450Sensors) {
          ds2450.begin(dsAddr[i]);
        }
        ds2450Sensors++;
        mqttSet(tochararray(cstr, "DS2450",tochararray(cpart, i)), tochararray(cmessage, addr));
      }
      else {
        DEBUG2_PRINT("Sensorfamilie nicht erkannt : 0x");
        DEBUG2_PRINTLN(dsAddr[i][0], HEX);
      }
    }
  }
  return true;
}

void ds1820Loop() {
  int i;
  ds18b20.requestTemperatures(); // Temp abfragen
  for (i = 0; i < MAX_DS_SENSORS; i++) {
    if (dsAddr[i][0] == 0x10 || dsAddr[i][0] == 0x28) {
      dsTemp[i] = ds18b20.getTempCByIndex(i);
      dsTime[i] = ntpTime.delta + millis() / 1000;
      char result[8]; // Buffer big enough for 7-character float
      char ds[6] = "temp0";
      dtostrf(dsTemp[i], 6, 1, result); // Leave room for too large numbers!
      if (i==0) {
        ds[4] = '\0';
        mqttSet(ds, result);
      } else {
        ds[4] = '0'+i;
        mqttSet(ds, result);
      }
      //DEBUG1_PRINT("temp"+String(i));
      //DEBUG1_PRINT(ds18b20.getTempCByIndex(i) );
      //DEBUG1_PRINTLN(" Grad Celsius");
    }
  }
}

void ds2450Loop() {
  ds2450.update();
  if (ds2450.isError()) {
    DEBUG1_PRINT("Error reading from DS2450 device");
  } else {
    for (int channel = 0; channel < 4; channel++) {
      mqttSet(tochararray(cstr, "AD",tochararray(cpart, channel)), tochararray(cmessage, ds2450.getVoltage(channel), 4, 1));
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
  delay(500);
}
void analogLoop() {
  analogWert = analogRead(A0);
  if (analogState == 0 && analogWert >= para.analog) {
    analogState = 1;
    mqttSet("analogIn", "1");
  } else if (analogState == 1 && analogWert < para.analog) {
    analogState = 0;
    mqttSet("analogIn", "0");
  }
}

void getData() {
  mqttSet("Heap", tochararray(cmessage, ESP.getFreeHeap()));
  yield();

  mqttSet("lwt", "up", true, 10);
  yield();

  if (bmpActive) {
    bmp280loop();
  yield();
  }
  if (bh1750Active) {
    bh1750loop();
  yield();
  }
  if (ds1820Sensors > 0) {
    ds1820Loop();
  yield();
  }
  if (ds2450Sensors > 0) {
    ds2450Loop();
  yield();
  }
}

void setAlarmLED() {
  if ((reedAlarmtoggle++ & 7) == 1){
    digitalWrite(ledPin, para.GpioLedOn);
  }else{
    digitalWrite(ledPin, !para.GpioLedOn);
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

void setPwm(byte nr, boolean set) {
  if (para.pin[nr] == PIN_PWM) {
    if (set) {
      digitalWrite(ledPin, para.GpioLedOn);
#     ifndef ESP32
        noTone(Pin[nr]);
#     else
        ledcWrite(0, 0);
#     endif
    } else {
      digitalWrite(ledPin, !para.GpioLedOn);
#     ifndef ESP32
        tone(Pin[nr],200,500);
        tone(Pin[nr],500,500);
        tone(Pin[nr],200,500);
#     else
        ledcWrite(0, 128);
#     endif
    }
  }
}

void setAlarm(){
  if ((reedSensor & reedAlarmstate) > 0) {
    // start local alarm
    timerAlarmstate.activate();
  } else if (reedAlarmstate == 0){
    timerAlarmstate.deactivate();
    digitalWrite(ledPin, !para.GpioLedOn);
  }
  for (byte i = 0; i < PIN_MAX; i++) {
    if (para.pin[i] == PIN_ACTOR) 
      setSwitch(i, reedActor & (1 << i));
  }
}

void Alarmloop(){
  int timer = millis();
  for (byte i = 0; i < PIN_MAX; i++) {
    if (para.pin[i] == PIN_SENSOR || para.pin[i] == PIN_ALARM) {
      int reedTemp = digitalRead(Pin[i]);
      if (reedTemp) {
        reedTemp = para.GpioOn[i];
      } else {
        reedTemp = para.GpioOn[i] == 0;
      }
      if ((reedTemp != pinState[i] && timer - reedTimer[i] > 100) | (timer - reedTimer[i] > 600000) | (timer < reedTimer[i]) | (reedTimer[i] == 0))
      {
        reedTimer[i] = timer;
        pinState[i] = reedTemp;
        mqttSet(tochararray(cstr, "reed",tochararray(cpart,i+1)), tochararray(cmessage, reedTemp));
        DEBUG1_PRINT("reed");
        DEBUG1_PRINT(i+1);
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
  if (para.analog){
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
  char msg[33];
  
  if (!mqtt) DEBUG1_PRINTLN("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      strcpy(msg,"I2C device found at address 0x");
      nDevices++;
    } else if (error == 4){
      strcpy(msg,"Unknown error    at address 0x");
    }
    msg[30] = hex[(address & 0xF0) >> 4];
    msg[31] = hex[address & 0x0F];
    msg[32] = '\0';
    addr[0] = msg[30];
    addr[1] = msg[31];
    if (error == 0 || error == 4){
      if (mqtt) {
        mqttSet(tochararray(cstr, "debug/i2c/",addr), msg, false);
      } else {
        DEBUG1_PRINTLN(msg);
      }
    }
  }
  if (nDevices == 0) {
      strcpy(msg,"No I2C devices found");
    if (mqtt) {
      mqttSet("debug/i2c/00", msg, false);
    } else {
      DEBUG1_PRINTLN(msg);
    }
  }
  else {
    if (!mqtt) DEBUG1_PRINTLN("Done.\n");
  }
}

void setConfig(byte nr, char receivedChar) {
  if (nr == 1){
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
    if (receivedChar == '5') {
      mqttSet("set/C1", "6");
      
      timerMqtt.deactivate();
      timerSensors.deactivate();
      timerReconnect.deactivate();
      timerNtp.deactivate();
      timerAlarmloop.deactivate();
#     ifdef ESP32
        WiFiClient wifiClient;
        t_httpUpdate_return ret = httpUpdate.update(wifiClient, para.mqtt_server, 80, "/esp8266/ota.php", tochararray(cstr, mVersionNr, mVersionBoard));
#     else      
        t_httpUpdate_return ret = ESPhttpUpdate.update(para.mqtt_server, 80, "/esp8266/ota.php", tochararray(cstr, mVersionNr, mVersionBoard));
#     endif
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          DEBUG1_PRINTLN("[update] Update failed: ");
          mqttSet("Update", "failed");
       
       break;
        case HTTP_UPDATE_NO_UPDATES:
          DEBUG1_PRINTLN("[update] Update no Update.");
          mqttSet("Update", "not necessary");
          break;
        case HTTP_UPDATE_OK:
          DEBUG1_PRINTLN("[update] Update ok."); // may not called we reboot the ESP
          mqttSet("Update", "ok");
          break;
        default:
          mqttSet("Update", tochararray(cmessage, ret));
          break;
      }
      timerMqtt.activate();
      timerSensors.activate();
      timerReconnect.activate();
      timerNtp.activate();
      timerAlarmloop.activate();
    }
    if (receivedChar == '7') {
      mqttSet("set/C1", "6");
      SPIFFS.format();
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
    bmp.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BME280::SAMPLING_X1,     /* Temp. oversampling */
                        Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                        Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                        Adafruit_BME280::STANDBY_MS_1000);
  }
  if (nr == 1){
    bmp.setSampling(Adafruit_BME280::MODE_FORCED,Adafruit_BME280::SAMPLING_X1,Adafruit_BME280::SAMPLING_X1,Adafruit_BME280::SAMPLING_X1,Adafruit_BME280::FILTER_OFF,Adafruit_BME280::STANDBY_MS_0_5);
  }
  if (nr == 2){
    bmp.setSampling(Adafruit_BME280::MODE_NORMAL,Adafruit_BME280::SAMPLING_X16,Adafruit_BME280::SAMPLING_X16,Adafruit_BME280::SAMPLING_X16,Adafruit_BME280::FILTER_OFF,Adafruit_BME280::STANDBY_MS_1000);
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

void set(char request, byte nr, char payload){
  if (request == 'C') {
    setConfig(nr, payload);
  } else if (request == 'O') {
    setOSS(nr, payload);
  } else if (request == 'S' && nr >= 0 && nr < PIN_MAX && para.pin[nr] == PIN_SWITCH) {
    setSwitch(nr, payload == '1');
  } else if (request == 'A' && nr >= 0 && nr < PIN_MAX && para.pin[nr] == PIN_ACTOR) {
    setActor(1 << nr, payload == '1');
  } else if (request == 'D') {
    setArmed(payload == '1');
  } else if (request == 'P' && nr >= 0 && nr < PIN_MAX) {
    setPwm(nr, payload == '1');
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
  cmessage[len+1] = '\0';
  char slash = topic[strlen(topic)-3];
  char request = topic[strlen(topic)-2];
  byte nr = topic[strlen(topic)-1] - '0';
  //DEBUG1_PRINT(slash);
  //DEBUG1_PRINT(request);
  //DEBUG1_PRINTLN(nr);
  //DEBUG1_PRINTLN(ack);
  if (slash == '/'){
    set(request, nr, receivedChar);
  } else if (String(topic).endsWith("/OSS")) {
    callbackOSS(payload, mLength);
  }
  yield();
  mqttSet("ack", cmessage);
}

void readInput() {
  if (Serial.available() > 0) {
    serialIn[serialPos++] = Serial.read();
    serialIn[serialPos] = 0;
  }
  // CR+LF, eventuell anpassen unter Linux ... == 0x0D
  if (serialIn[serialPos-1] == 0x0A){
    serialPos = 0;
    char request = serialIn[0];
    byte nr= serialIn[1] - '0';
    char payload= serialIn[2];
    
    if (request == 'c') {
      //Einstellen();
      mqttReconnect();  
    } else if (request == 'f') {
#ifndef ESP32
      FSInfo fs_info;
      Serial.println("Please wait 30 secs for SPIFFS to be formatted");
      SPIFFS.format();
      Serial.println("Spiffs formatted");
      //See more at: http://www.esp8266.com/viewtopic.php?f=29&t=8194#sthash.mj02URAZ.dpuf
      SPIFFS.info(fs_info);
      DEBUG1_PRINT("totalBytes ");
      DEBUG1_PRINT(fs_info.totalBytes);
#endif
    } else if (request == 'i') {
      i2cScan();
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
    } else if (request == 'A' || request == 'C' || request == 'D' || request == 'S' || request == 'O' || request == 'P') { 
      set(request, nr, payload);
    }
  }
}

void setupPinmode(){
  for (byte i = 0; i < PIN_MAX; ++i) {
    //DEBUG1_PRINT(tochararray(cstr, pinState[i]));
    //DEBUG1_PRINT(tochararray(cstr, "INPUT_PULLUP ",tochararray(cpart, pinState[i])));
    if (para.pin[i] == PIN_SENSOR || para.pin[i] == PIN_ALARM) {
      pinMode(Pin[i], INPUT_PULLUP);
      pinState[i] = para.GpioOn[i] == 0; //ok
      mqttSet(PinName[i], tochararray(cstr, "INPUT_PULLUP ",tochararray(cpart, pinState[i])));
    }else if (para.pin[i] == PIN_ACTOR || para.pin[i] == PIN_SWITCH || para.pin[i] == PIN_LED) {
      pinMode(Pin[i], OUTPUT);
      // default: Switch off, LED off
      pinState[i] = !para.GpioOn[i];
      digitalWrite(Pin[i], pinState[i]);
      mqttSet(PinName[i], tochararray(cstr, "OUTPUT ",tochararray(cpart, pinState[i])));
    }else if (para.pin[i] == PIN_PWM) {
#     ifdef ESP32
        ledcSetup(0, 1000, 8);
        ledcAttachPin(Pin[i], 0);
#     endif
      mqttSet(PinName[i], "PWM");
    } else {
      mqttSet(PinName[i], tochararray(cstr, "UNDEF ",tochararray(cpart, para.pin[i])));
    }
  }
  pinMode(A0, INPUT);
}

void setupI2c(boolean rescan){
  if (para.i2c){
    if (!bmpActive){
      Wire.begin(sdaPin, sclPin); 
      // in bmp.begin() keine Pinzuordnung möglich
      // bmpActive = bmp.begin(sdaPin, sclPin);
      bmpActive = bmp.begin();
      bmpTyp = bmp.sensorID();
      bmeActive = (bmpTyp == 0x60);
      if (bmpActive) {
        DEBUG1_PRINT("BMP init success! ");
        mqttSet("BMP-Typ", tochararray(cstr, bmpTyp));//"+0x"+String(bmpTyp, HEX));
        DEBUG1_PRINT("Typ "); DEBUG1_PRINTLN(bmpTyp, HEX);
        bmp.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BME280::SAMPLING_X1,     /* Temp. oversampling */
                        Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                        Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                        Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                        Adafruit_BME280::STANDBY_MS_1000);
       } else {
        DEBUG1_PRINT("BMP failed! ");
        DEBUG1_PRINT("Typ "); DEBUG1_PRINTLN(bmpTyp, HEX);
        mqttSet("BMP-Typ", tochararray(cstr, bmp.sensorID()));//"-0x"+String(bmp.sensorID(), HEX));
       }
    }
    if (!bh1750Active){
      bh1750Active = bh1750.begin();
      if (bh1750Active) {
        DEBUG1_PRINTLN("Lightsensor init success!");
      }
    }
  }
  if (!para.i2c){
    DEBUG1_PRINTLN("BMP/BH1750 init failed!");
  } else if (!bmpActive and !bh1750Active) {
    DEBUG1_PRINTLN("BMP/BH1750 init failed!");
  }
  if (rescan && para.i2c) {
    DEBUG1_PRINTLN("I2C Scanner");
    i2cScan();
  }
}

void setup1wire(boolean rescan){
  if (para.onewire) {
    if (!dsActive){
      dsActive = dsSetup(rescan);
    }
  }
  if (!dsActive)
    DEBUG1_PRINTLN("no 1wire");
}

void setup(){
  Serial.begin(9600);
  DEBUG1_PRINTLN();
  DEBUG1_PRINTLN(board);
  // getSystem();
  getPara();
  // byte i;
  DEBUG1_PRINTLN("getPara");

  setupPinmode();
  DEBUG1_PRINTLN("setupPinmode");
  setupI2c(false);
  DEBUG1_PRINTLN("setupI2c");
  setup1wire(false);
  DEBUG1_PRINTLN("setup1wire");
  timerAlarmloop.begin(1000, Alarmloop);
  DEBUG1_PRINTLN("timerAlarmloop");
  timerSensors.begin(para.timerMsec[1], getData);
  DEBUG1_PRINTLN("timerSensors");

#ifndef ESP32
  stationDisconnectedHandler = WiFi.onStationModeDisconnected(onDisconnect);
  stationGotIpHandler = WiFi.onStationModeGotIP(onGotIP);
#endif
  wifiSetup(para.pVersion > 0);// && ( para.checksum == 123456 || para.checksum = 999999)
  DEBUG1_PRINTLN("wifiSetup");
  httpSetup();
  DEBUG1_PRINTLN("httpSetup");

  if (wifiStation){
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
}

void loop(){
  timerAlarmloop.update();
  yield();
  timerAlarmstate.update();
  yield();
  timerSensors.update();
  yield();
  timerRestartDelay.update();
  yield();
  if (wifiStation){
    timerMqtt.update();
    yield();
    timerReconnect.update();
    yield();
    timerNtp.update();
  }
  yield();
  mqttLoop();
  yield();
  http.handleClient();
  yield();
  readInput();
}
