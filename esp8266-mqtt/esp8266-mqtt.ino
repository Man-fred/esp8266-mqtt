#include <ESP8266WiFi.h>
// enables OTA updates
#include <ESP8266httpUpdate.h>
// enables webconfig
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
// enables storing webpages in EEPROM, not in sketch
#include <FS.h>
// enables storing config-data in EEPROM
#include <EEPROM.h>
// enables NTP / RTC
#include <WiFiUdp.h>

#define MQTT_MAX_PACKET_SIZE 256
#include <PubSubClient.h>
#include <Wire.h>
#include <BMP280.h>
#include <BH1750FVI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*  V01-00 : Arduino 1.6.7
 *  V01-01 : Arduino 1.6.10, DallasTemperature, OneWire updated
 *  V01-04 : Arduino 1.6.10, webserver for configdate, saved in eeprom, detection of NodeMCU and WeMos D1 mini
 *  V01-05 : Arduino 1.8.2,  
 *  V01-06 : Arduino 1.8.9,  
 */
String mVersionNr = "V01-06-14.esp8266-mqtt.ino.";
#ifndef DBG_OUTPUT_PORT
  #define DBG_OUTPUT_PORT Serial
#endif
#define DEBUG 2
/*
   Wire - I2C Scanner

   The NodeMcu / WeMos D1 Mini I2C bus uses pins:
   D1 (5)= SCL
   D2 (4)= SDA
*/
const int sclPin = D1;
const int sdaPin = D2;
/*
 *  Pin with LED on Wemos d1 mini D4 (2), Nodemcu D0 (16)
 */
const byte ledPin = BUILTIN_LED; 
/*
 * ARDUINO_* equals to ARDUINO_<...build.board from boards.txt>
 * OneWire-Bus D4 (2), collision with builtin LED on WeMos D1 Mini (2)
 * change to   D3 (0) 
 */
#ifdef ARDUINO_ESP8266_NODEMCU
  const byte board = 1;
  const byte ONE_WIRE_BUS = D4;
  const byte reed1In = D7;
  const byte reed2In = D6;
  const byte S2Pin = D3; 
  const byte S3Pin = D5; 
  const byte S4Pin = D8; 
  String mVersionBoard = "nodemcu";
#elif ARDUINO_ESP8266_WEMOS_D1MINI
  const byte board = 2;
  const byte ONE_WIRE_BUS = D3;
  const byte reed1In = D7;
  const byte reed2In = D6;
  const byte S2Pin = D0; 
  const byte S3Pin = D5; 
  const byte S4Pin = D8; 
  String mVersionBoard = "d1_mini";
#else
  const byte board = 3;
  const byte ONE_WIRE_BUS = D4;
  const byte reed1In = D7;
  const byte reed2In = D6;
  const byte S2Pin = D3; 
  const byte S3Pin = D5; 
  const byte S4Pin = D8; 
  String mVersionBoard = "unknown";
#endif


// Parameters for WiFi and MQTT
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
  
  int timerMsec[3];
  bool reed1;
  bool reed2;
  int analog;
  unsigned int checksum;
};
// noch in Parameter aufnehmen
bool GpioRelaisOn = false;
bool GpioLedOn = false;
bool GpioReedOn = false;

/* Set the tm_t fields for the local time. */
struct NtpTime {
  unsigned long epoch, isdst, sec, min, hour, wday, leap, year, mon, yday, mday, startup, delta;
};

struct NtpTime ntpTime;

struct Parameter para;

//String mChipId = "";
//String mClient = "";
unsigned int mFlashSize = 0;

//========= Variables for sketch-part "Http" ===============
WiFiClient espClient;
ESP8266WebServer http(80);
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

BMP280 bmp;
boolean bmpActive = true;
boolean bmeActive = true;
char bmpTyp;
double bmpT;
double bmpP;
double bmeH;
unsigned long bmpTime;
//#define bmpP0 1013.25

BH1750FVI bh1750;
boolean bh1750Active = true;
uint16_t bh1750Lux = 0;
unsigned long bh1750Time;

OneWire ds(ONE_WIRE_BUS); /* Ini oneWire instance */
DallasTemperature ds18b20(&ds);/* Dallas Temperature Library für Nutzung der oneWire */
#define MAX_DS_SENSORS 2
boolean dsActive = true;
int dsTempSensors = 0;
byte dsAddr[MAX_DS_SENSORS][8];
float dsTemp[MAX_DS_SENSORS];
unsigned long dsTime[MAX_DS_SENSORS];

int reed1State = 0; // undefined
int reed1Timer = 0; // undefined
int reed2State = 0; // undefined
int reed2Timer = 0; // undefined
int reedAlarmstate = 0; // 0: not armed, 1: armed
int reedActor = 0;      // 0: no action, 1: internal, 2: light, 3: external sirene
int reedState = 0; // undefined
byte reedAlarmtoggle = 0;

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

#define MAX_MESSAGES 20

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
          aCallback();
        }
      }
    }
};

MyTimer timerMqtt; // mqtt delay
MyTimer timerSensors; // collect sensors delay
MyTimer timerReconnect; // reconnect wifi delay
MyTimer timerNtp; // ntp-loop 
MyTimer timerRestart; // restart delay in websession
MyTimer timerAlarmloop; // alarm sensors delay
MyTimer timerAlarmstate; // blink LED for local alarm

String mPayloadKey[MAX_MESSAGES];
String mPayloadValue[MAX_MESSAGES];
boolean mPayloadRetain[MAX_MESSAGES];
int mPayloadSet = 0;
int mPayloadPublish = 0;

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

void getPara() {
  EEPROM.begin(4096);
  EEPROM.get(0, para);
  if (para.pVersion > 0) {
    DEBUG_PRINTLN("Flash loaded");
    if (para.checksum != 123456) {
      para.checksum = 999999;
    }
    if (para.timerMsec[0] < 100) para.timerMsec[0] = 1000;
    if (para.timerMsec[1] < 1000) para.timerMsec[1] = 60000;
    if (para.timerMsec[2] < 1000) para.timerMsec[2] = 20000;
  } else {
    // Default Parameter setzen //
    para.pVersion = 0;
    strncpy( para.ssid, "...", 20); para.ssid[20 - 1] = '\0';
    
    strncpy( para.mqtt_server, "192.168.178.60", 20); para.mqtt_server[20 - 1] = '\0';
    para.mqtt_port = 1883;
    strncpy( para.mClient, "ESP", 20); para.mClient[20 - 1] = '\0';
    strncpy( para.mPre, "esp/", 10); para.mPre[10 - 1] = '\0';
    strncpy( para.mSub, "set/+", 10); para.mSub[10 - 1] = '\0';
    strncpy( para.mLwt, "lwt", 10); para.mLwt[10 - 1] = '\0';
    
    // timerMqtt: MQTT - Sendpipe 
    para.timerMsec[0] =  1000;
    para.timerMsec[1] = 60000;
    para.timerMsec[2] = 20000;
    para.reed1 = false;
    para.reed2 = false;
    para.analog = 0;
    para.checksum = 111111;

    // 512k: 3c000 - 3ffff
    // 1024k 7c000
    // 2048/4096: 7c000 (max Flashsize 428k) oder fc000 (max Flashsize 940k)
    // 4096: zusaetzlich 1fc000 - 3fbfff (>= 2048 KB)
    EEPROM.put(0, para);
    EEPROM.commit();            // EEPROM Schreiben
    DEBUG_PRINTLN("Flash written");
    // */
  }
  EEPROM.end();
}

void mqttSet(String key, String value, boolean retain = true) {
  mPayloadKey[mPayloadSet] = String(para.mPre);
  mPayloadKey[mPayloadSet] += para.mClient;
  mPayloadKey[mPayloadSet] += "/";
  mPayloadKey[mPayloadSet] += key;
  mPayloadValue[mPayloadSet] = value;
  mPayloadRetain[mPayloadSet] = retain;
  DEBUG3_PRINTLN("mqttSet  " + String(mPayloadSet) + ": " + mPayloadKey[mPayloadSet] + " <" + value + ">");
  mPayloadSet++;
  if (mPayloadSet > MAX_MESSAGES-1) {
    mPayloadSet = 0;
  }
}
// Sends a payload to the broker
void mqttSend() {
  if (mPayloadKey[mPayloadPublish] != "") {
    client.publish(mPayloadKey[mPayloadPublish].c_str(), mPayloadValue[mPayloadPublish].c_str(), mPayloadRetain[mPayloadSet]);
    DEBUG3_PRINTLN("mqttSend " + String(mPayloadPublish) + ": " + mPayloadKey[mPayloadPublish] + " <" + mPayloadValue[mPayloadPublish] + ">");
    mPayloadKey[mPayloadPublish] = "";
    mPayloadPublish++;
    if (mPayloadPublish > MAX_MESSAGES-1) {
      mPayloadPublish = 0;
    }
  }
}

void bh1750loop() {
  bh1750Lux = bh1750.readLightLevel();
  bh1750Time = ntpTime.delta + millis() / 1000;
  mqttSet("lux", String(bh1750Lux));

  DEBUG1_PRINT("Light: ");
  DEBUG1_PRINT(bh1750Lux);
  DEBUG1_PRINTLN(" lx");
}

void bmp280loop() {
  char result = bmp.startMeasurment();

  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(bmpT, bmpP, bmeH);

    if (result != 0) {
      if (dsTempSensors > 0) {
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

      DEBUG1_PRINT("T = \t"); DEBUG_PRINT(bmpT, 2); DEBUG_PRINT(" degC\t");
      DEBUG1_PRINT("P = \t"); DEBUG_PRINT(bmpP, 2); DEBUG_PRINT(" mBar\t");
      DEBUG1_PRINT("H = \t"); DEBUG_PRINT(bmeH, 2); DEBUG_PRINTLN(" %");
      //DEBUG1_PRINT("A = \t");DEBUG_PRINT(A,2); DEBUG_PRINTLN(" m");

    }
    else {
      DEBUG1_PRINTLN("BMP no result.");
    }
  }
  else {
    DEBUG1_PRINTLN("BMP no Start.");
  }
}

boolean dsSetup() {
  byte i,j;
  byte present = 0;
  byte data[12];
  String addr = "";
  ds18b20.begin();

  for (i = 0; i < MAX_DS_SENSORS; i++) {
    if (!ds.search(dsAddr[i]))
    {
      if (i == 0) {
        DEBUG_PRINTLN("No OneWire addresses.");
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
        DEBUG2_PRINT("Sensor ist aus der DS18S20 Familie.\n");
        dsTempSensors++;
        mqttSet("DS18S20"+String(i), addr);
      }
      else if ( dsAddr[i][0] == 0x28) {
        DEBUG2_PRINT("Sensor ist aus der DS18B20 Familie.\n");
        dsTempSensors++;
        mqttSet("DS18B20"+String(i), addr);
      }
      else {
        DEBUG2_PRINT("Sensorfamilie nicht erkannt : 0x");
        DEBUG2_PRINTLN(dsAddr[i][0], HEX);
      }
    }
  }
  return true;
}

void dsLoop() {
  int i;
  ds18b20.requestTemperatures(); // Temp abfragen
  for (i = 0; i < MAX_DS_SENSORS; i++) {
    if (dsAddr[i][0] == 0x10 || dsAddr[i][0] == 0x28) {
      dsTemp[i] = ds18b20.getTempCByIndex(i);
      dsTime[i] = ntpTime.delta + millis() / 1000;
      if (i==0) {
        mqttSet("temp", String(dsTemp[i], 2));
      } else {
        mqttSet("temp"+String(i), String(dsTemp[i], 2));
      }
      DEBUG1_PRINT("temp"+String(i));
      DEBUG1_PRINT(ds18b20.getTempCByIndex(i) );
      DEBUG1_PRINTLN(" Grad Celsius");
    }
  }
}

void analogLoop() {
  analogWert = analogRead(A0);
  if (analogState == 0 && analogWert >= para.analog) {
    analogState = 1;
    mqttSet("analogIn", String(1));
  } else if (analogState == 1 && analogWert < para.analog) {
    analogState = 0;
    mqttSet("analogIn", String(0));
  }
}

void getData() {
  mqttSet("Heap", String(ESP.getFreeHeap()));

  if (bmpActive) {
    bmp280loop();
  }
  if (bh1750Active) {
    bh1750loop();
  }
  if (dsActive) {
    dsLoop();
  }
}

void setAlarmLED() {
  if ((reedAlarmtoggle++ & 7) == 1){
    digitalWrite(ledPin, GpioLedOn);
  }else{
    digitalWrite(ledPin, !GpioLedOn);
  }
  //DEBUG1_PRINTLN("reedLED "+String(reedAlarmtoggle & 3)+" "+String(reedAlarmtoggle));
}

void setAlarm(){
  if ((reedState > 0 && reedAlarmstate > 0) || reedActor > 0) {
    // start local alarm
    timerAlarmstate.activate();
  } else {
    timerAlarmstate.deactivate();
    digitalWrite(ledPin, !GpioLedOn);
  }
  digitalWrite(S2Pin, (reedActor & 1) ? GpioRelaisOn : !GpioRelaisOn);
  digitalWrite(S3Pin, (reedActor & 2) ? GpioRelaisOn : !GpioRelaisOn);
  digitalWrite(S4Pin, (reedActor & 4) ? GpioRelaisOn : !GpioRelaisOn);
}

void setReed(int nr, int state) {
  mqttSet("reed"+String(nr), String(state));
  DEBUG1_PRINTLN("reed"+String(nr)+" "+String(state));
  if (nr > 1) 
    nr = (1 << (nr-1));
  if (state == 0) {
    reedState = reedState & ~nr;
  }else {
    reedState = reedState | nr;
  }
  setAlarm();
  //DEBUG1_PRINTLN("reedstate "+String(reedState));
}

void getAlarm(){
  if (para.reed1) {
    int reed1Temp = digitalRead(reed1In);
    if ((reed1Temp != reed1State && millis() - reed1Timer > 100) | (millis() - reed1Timer > 600000))
    {
      reed1Timer = millis();
      reed1State = reed1Temp;
      setReed(1,reed1State);
    }
  }
  if (para.reed2) {
    int reed2Temp = digitalRead(reed2In);
    if ((reed2Temp != reed2State && millis() - reed2Timer > 100) | (millis() - reed2Timer > 600000))
    {
      reed2Timer = millis();
      reed2State = reed2Temp;
      setReed(2,reed2State);
    }
  }
  if (para.analog){
    analogLoop();
  }
}

void espRestart() {
  ESP.restart();
}

void i2cScan(boolean mqtt = false) {
  byte error, address;
  int nDevices;
  String msg,addr;
  
  if (!mqtt) DEBUG_PRINTLN("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    addr = (address < 16 ? "0" : "") + String(address, HEX);
    
    if (error == 0){
      msg = "I2C device found at address 0x";
      nDevices++;
    } else if (error == 4){
      msg = "Unknown error at address 0x";
    }
    if (error == 0 || error == 4){
      if (mqtt) {
        mqttSet("debug/i2c/"+addr, msg+addr, false);
      } else {
        DEBUG_PRINTLN(msg+addr);
      }
    }
  }
  if (nDevices == 0) {
    msg = "No I2C devices found";
    if (mqtt) {
      mqttSet("debug/i2c/00", msg, false);
    } else {
      DEBUG_PRINTLN(msg);
    }
  }
  else {
    if (!mqtt) DEBUG_PRINTLN("Done.\n");
  }
}

void callbackMSub(byte* payload, unsigned int mLength) {
  for (int i = 0; i < mLength; i++) {
    char receivedChar = (char)payload[i];
    DEBUG_PRINT(receivedChar);
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
      espRestart();
    }
    if (receivedChar == '5') {
      mqttSet("set/S1", "6");

      timerMqtt.deactivate();
      timerSensors.deactivate();
      timerReconnect.deactivate();
      timerNtp.deactivate();
      timerAlarmloop.deactivate();
      t_httpUpdate_return ret = ESPhttpUpdate.update(para.mqtt_server, 80, "/esp8266/ota.php", (mVersionNr+mVersionBoard).c_str());
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          DEBUG_PRINTLN("[update] Update failed: "+ mVersionNr+mVersionBoard);
          mqttSet("Update", mVersionNr+mVersionBoard + " failed");
          break;
        case HTTP_UPDATE_NO_UPDATES:
          DEBUG_PRINTLN("[update] Update no Update.");
          mqttSet("Update", mVersionNr+mVersionBoard + " not necessary");
          break;
        case HTTP_UPDATE_OK:
          DEBUG_PRINTLN("[update] Update ok."); // may not called we reboot the ESP
          mqttSet("Update", mVersionNr+mVersionBoard + " ok");
          break;
        default:
          mqttSet("Update", mVersionNr+mVersionBoard + " unknown error " + String(ret));
          break;
      }
      timerMqtt.activate();
      timerSensors.activate();
      timerReconnect.activate();
      timerNtp.activate();
      timerAlarmloop.activate();
    }
    if (receivedChar == 'i') {
      i2cScan(true);
    }
  }
}

void callbackS2(byte* payload) {
  char receivedChar = (char)payload[0];
  if (receivedChar == '1') {
    digitalWrite(S2Pin, GpioRelaisOn);
  }
  if (receivedChar == '0') {
    digitalWrite(S2Pin, !GpioRelaisOn);
  }
}

void setA1(byte* payload) {
  char receivedChar = (char)payload[0];
  if (receivedChar == 'a') {
    reedAlarmstate = 1;
  }
  if (receivedChar == 'd') {
    reedAlarmstate = 0;
  }
  setAlarm();
}

void setA2(byte* payload) {
  char receivedChar = (char)payload[0];
  if (receivedChar == '0') {
    reedActor = reedActor & ~1;
  }
  if (receivedChar == '1') {
    reedActor = reedActor | 1;
  }
  setAlarm();
}
void setA3(byte* payload) {
  char receivedChar = (char)payload[0];
  if (receivedChar == '0') {
    reedActor = reedActor & ~2;
  }
  if (receivedChar == '1') {
    reedActor = reedActor | 2;
  }
  setAlarm();
}
void setA4(byte* payload) {
  char receivedChar = (char)payload[0];
  if (receivedChar == '0') {
    reedActor = reedActor & ~4;
  }
  if (receivedChar == '1') {
    reedActor = reedActor | 4;
  }
  setAlarm();
}

void callbackOSS(byte* payload, unsigned int mLength) {
  if (mLength == 1 and payload[0] < 5) {
    bmp.setOversamplingP(payload[0]);
  }
}
  
void callback(char* topic, byte* payload, unsigned int mLength) {
  String ack = "[" + String(topic) + "] "+String((const char*) payload).substring(0, mLength);
  DEBUG_PRINTLN(ack);

  if (String(topic).endsWith("/S1")) {
    callbackMSub(payload, mLength);
  } else if (String(topic).endsWith("/S2")) {
    callbackS2(payload);
  } else if (String(topic).endsWith("/A1")) {
    setA1(payload);
  } else if (String(topic).endsWith("/A2")) {
    setA2(payload);
  } else if (String(topic).endsWith("/A3")) {
    setA3(payload);
  } else if (String(topic).endsWith("/A4")) {
    setA4(payload);
  } else if (String(topic).endsWith("/OSS")) {
    callbackOSS(payload, mLength);
  }
  mqttSet("ack", ack);
}

void readInput() {
  char inser = Serial.read();
  if (inser == 'c') {
    //Einstellen();
    mqttReconnect();  
  } else if (inser == 'f') {
    FSInfo fs_info;
    Serial.println("Please wait 30 secs for SPIFFS to be formatted");
    SPIFFS.format();
    Serial.println("Spiffs formatted");
    //See more at: http://www.esp8266.com/viewtopic.php?f=29&t=8194#sthash.mj02URAZ.dpuf
    SPIFFS.info(fs_info);
    Serial.println("totalBytes " + String(fs_info.totalBytes));
  } else if (inser == 'i') {
    i2cScan();
  } else if (inser == 'l') {
    listSpiffs();
  } else if (inser == 'w') {
    Serial.println("");
    Serial.println("Mit Wlan verbunden");
    Serial.print("IP Adresse: ");
    Serial.println(WiFi.localIP());
    //Serial.println("Zeit: " + PrintDate(now()) + " " + PrintTime(now()));
    //printUser();
    Serial.println(getConfig());
    Serial.println(getIndex());
  }
}

void setup(){
  Serial.begin(9600);
  DEBUG_PRINTLN();
  DEBUG_PRINTLN(board);
  getSystem();
  getPara();
  pinMode(ledPin, OUTPUT);
  pinMode(reed1In, INPUT_PULLUP);
  pinMode(reed2In, INPUT_PULLUP);
  pinMode(S2Pin, OUTPUT);
  pinMode(S3Pin, OUTPUT);
  pinMode(S4Pin, OUTPUT);
  pinMode(A0, INPUT);
  // default: LED off, Relais off
  digitalWrite(ledPin, HIGH);
  digitalWrite(S2Pin, HIGH);
  
  bmpActive = bmp.begin(sdaPin, sclPin);
  if (bmpActive) {
    DEBUG_PRINTLN("BMP init success!");
    bmeActive = ((bmpTyp = bmp.getChipId()) == 0x60);
    DEBUG1_PRINT("Typ "); DEBUG1_PRINT(bmpTyp);
    //bmp.setOversamplingAll(1,1,1);
  }
  bh1750Active = bh1750.begin();
  if (bh1750Active) {
    DEBUG_PRINTLN("Lightsensor init success!");
  }
  if (!bmpActive and !bh1750Active) {
    DEBUG_PRINTLN("BMP/BH1750 init failed!");
    DEBUG_PRINTLN("I2C Scanner");
    // Wire.begin(sdaPin, sclPin); in bmp.begin()
    i2cScan();
  }
  dsActive = dsSetup();
  timerAlarmloop.begin(1000, getAlarm);
  timerSensors.begin(para.timerMsec[1], getData);
  
  if (para.pVersion > 0){ // && ( para.checksum == 123456 || para.checksum = 999999) {
    //WiFi.setPhyMode(WIFI_PHY_MODE_11G);
    //WiFi.setOutputPower(20.5);
    wifiStaSetup();
  } else {
    wifiAPSetup();
  }
  httpSetup();
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
  timerAlarmstate.begin(333, setAlarmLED, false);
}

void loop(){
  timerAlarmloop.update();
  yield();
  timerAlarmstate.update();
  yield();
  timerSensors.update();
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
