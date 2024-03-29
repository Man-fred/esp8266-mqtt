
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
