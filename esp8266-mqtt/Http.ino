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
const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><input type='submit' value='Upload'></form><p><a href=\"/home.htm\">Goto Home</a></p>";

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
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    DEBUG2_PRINT("handleFileRead: " + path);
    File file = SPIFFS.open(path, "r");
    size_t sent = http.streamFile(file, contentType);
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
      fsUploadFile = SPIFFS.open(filename, "w");
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
    fsUploadFile = SPIFFS.open(filename, "w");
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
  if(!SPIFFS.exists(path))
    return http404();
  SPIFFS.remove(path);
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
  if(SPIFFS.exists(path))
    return http.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
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
  Dir dir = SPIFFS.openDir(path);
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
    for (i = 0; i < 3; i++) {
      json += "\",\"timerMsec"+String(i)+"\":\""+ String(para.timerMsec[i]);
    }
    for (i = 0; i < PIN_MAX; i++) {
      json += "\",\"PinName"+String(i)+"\":\""+ PinName[i];
      json += "\",\"pin"+String(i)+"\":\""+ String(para.pin[i]);
      json += "\",\"GpioOn"+String(i)+"\":\""+ String(para.GpioOn[i]);
    }
    json += "\",\"GpioLedOn\":\""+ String(para.GpioLedOn);
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
    if (bh1750Active) {
      json += "\",\"bh1750Lux\":\""+String(bh1750Lux);
      json += "\",\"bh1750Time\":\""+fillNtpTime(bh1750Time);
    }
    json += "\",\"ds\":\""+String(ds1820Sensors > 0);
    if (ds1820Sensors > 0) {
      for (i = 0; i < MAX_DS_SENSORS; i++) {
        if (dsAddr[i][0] == 0x10 || dsAddr[i][0] == 0x28) {
          json += "\",\"dsT"+String(i)+"\":\""+String(dsTemp[i], 2);
          json += "\",\"dsTime"+String(i)+"\":\""+fillNtpTime(dsTime[i]);
        }
      }
    }
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
    bool isHidden;

    char * json = (char *) malloc (1000);
    strcpy (json, "[\n");
#ifndef ESP32
    char network[100];
    for (int i = 0; i < ap_count; i++)
    {
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel, isHidden);
      sprintf(network, "{\"SSID\":\"%s\",\"channel\":\"%s\",\"RSSI\":\"%d\",\"encryption\":\"%s\",\"hidden\":\"%s\" }\n", ssid.c_str(), channel, RSSI, encryptionType == ENC_TYPE_NONE ? " " : "*", isHidden ? "hidden" : "");
      if (strlen(json) < 899)
        strcat(json, network);
    }
#endif
    strcat(json, "]");
    return(json);
}
  
  /* Root page for the webserver */
void httpHome() {
  // Check if there are any GET parameters
  if (http.hasArg("restart")) {
    // muss in timer, sonst http-Timeout und endlos-Reset
    timerRestartDelay.begin(5000, restartDelay);
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
    para.timerMsec[0] = http.arg("timerMsec0").toInt();
    para.timerMsec[1] = http.arg("timerMsec1").toInt();
    para.timerMsec[2] = http.arg("timerMsec2").toInt();
    for (byte i = 0; i < PIN_MAX; i++) {
      para.pin[i] = http.arg("pin"+String(i)).toInt();
      para.GpioOn[i] = http.arg("GpioOn"+String(i)).toInt();
    }
    para.GpioLedOn = http.arg("GpioLedOn").toInt();
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
void listSpiffs(){
    DBG_OUTPUT_PORT.setDebugOutput(true);
#ifndef ESP32
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      DEBUG1_PRINTF("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    DEBUG1_PRINTF("\n");
#endif
}
void httpSetup() {
  SPIFFS.begin();
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
    bool isHidden;

    String json = "[";
    for (int i = 0; i < ap_count; i++)
    {
#ifndef ESP32
      WiFi.getNetworkInfo(i, ssid, encryptionType, RSSI, BSSID, channel, isHidden);
      //sprintf(network, "{\"SSID\":\"%s\",\"channel\":\"%s\",\"RSSI\":\"%d\",\"encryption\":\"%s\",\"hidden\":\"%s\" }\n", ssid.c_str(), channel, RSSI, encryptionType == ENC_TYPE_NONE ? " " : "*", isHidden ? "hidden" : "");
      if (i > 0)
        json += ",\n";  
      String network = "{\"SSID\":\""+ssid+"\",\"channel\":\""+String(channel)+
            "\",\"RSSI\":\""+String(RSSI)+"\",\"encryption\":\""+(encryptionType == ENC_TYPE_NONE ? " " : "*")+
            "\",\"hidden\":\""+(isHidden ? "hidden" : "")+"\" }";
      //json += %s%s%d%s%s, , channel, RSSI, encryptionType == ENC_TYPE_NONE ? " " : "*", isHidden ? "hidden" : "");
      json += network;
      DEBUG3_PRINTLN(network);
#else
      sprintf(json, "%s{\"SSID\":\"%s\",\"channel\":\"%s\",\"RSSI\":\"%d\",\"encryption\":\"%s\",\"hidden\":\"%s\" }\n", json, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " " : "*", "");
#endif
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
