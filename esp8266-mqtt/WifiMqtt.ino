void wifiSetup(boolean station) {
  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_OFF);

  if (station){
    wifiStaSetup();
  } else {
    wifiAPSetup();
  }
}
void wifiStaSetup() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    wifiStation = false;
    int i = 0;
    DEBUG_PRINT("WiFi.status() ");
    DEBUG_PRINT(WiFi.status());
    WiFi.begin(para.ssid, para.password);
    // 60 Sekunden auf WLan warten
    while (WiFi.status() != WL_CONNECTED && i++ < 100) {
      delay(600);
      DEBUG_PRINT(".");
    }
    if (i >= 100){
      // Erreichbar über AP 192.168.4.1 für 5 Min., anschließend Neustart im bekannten WLAN
      wifiAPSetup();
      timerRestart.begin(300000, espRestart);
    } else {
      DEBUG_PRINT(", IP: ");
      DEBUG_PRINT(WiFi.localIP());
      DEBUG_PRINT(", MAC: ");
      WiFi.macAddress(para.mMac);                   //get MAC address of interface
      DEBUG_PRINT(macToString(para.mMac));
      DEBUG_PRINT(", Client: ");
      DEBUG_PRINT(para.mClient);
      DEBUG_PRINTLN(" ok");
      WiFi.setAutoReconnect(true);
      wifiStation = true;
      mqttSet("IP", WiFi.localIP().toString());
      mqttSet("RSSI", String(WiFi.RSSI()));
    }
  } else {
    wifiStation = true;
  }
}

void wifiAPSetup()
{
  wifiStation = false;
  WiFi.mode(WIFI_AP);
  DEBUG_PRINT("WiFi AP ...");
  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String AP_NameString = macToEsp(mac);
  DEBUG_PRINT(AP_NameString);

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, "123456789");
  DEBUG_PRINT(", pass 123456789");
  DEBUG_PRINTLN(" ok"); 
  delay(500);                   //Abwarten 0,5s
  DEBUG_PRINT("IP Adresse ");  //Ausgabe aktueller IP des Servers
  DEBUG_PRINTLN(WiFi.softAPIP());
}

void onGotIP(const WiFiEventStationModeGotIP& event){
  /*mqttSet("connect", "gotIp", false);
  Serial.print("Station connected, IP: ");
  Serial.println(WiFi.localIP());*/
};

void onDisconnect(const WiFiEventStationModeDisconnected& event){
  mqttSet("connect", "disconnected "+(String)event.reason, false);
  if (WiFi.status() == WL_CONNECTED) {
    // See https://github.com/esp8266/Arduino/issues/5912
    WiFi.disconnect();
  }
  wifiStation = false;
  Serial.println("Station disconnected");
};

void mqttSetup(){
  DEBUG_PRINTLN(para.mqtt_server);
  client.setServer(para.mqtt_server, para.mqtt_port);
  client.setCallback(callback);
}
void mqttLoop(){
  if (client.connected()) {
    client.loop();
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
  mqttSet("mqttConnected", String(client.state()));
}

/*  String macID = String(mac[WL_MAC_ADDR_LENGTH - 3], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
*/
String macToEsp(const unsigned char* mac) {
  char buf[10];
  snprintf(buf, sizeof(buf), "ESP%02x%02x%02x", mac[3], mac[4], mac[5]);
  String erg = String(buf);
  erg.toUpperCase();
  return erg;
}
String macToString(const unsigned char* mac) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}
void mqttReconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    // MQTT disconnected
    mqttFailed();
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi disconnected
      wifiStaSetup();
    }
    if (wifiStation){
      DEBUG_PRINT("MQTT...");
      // Attempt to connect
      // boolean connect (clientID, username, password, willTopic, willQoS, willRetain, willMessage)
      String lSubscribe = para.mPre;
      lSubscribe += para.mClient;
      lSubscribe += "/";
      String lLwt = lSubscribe + para.mLwt;
      lSubscribe += para.mSub;
      if (client.connect(para.mClient, para.mqtt_user, para.mqtt_pass, lLwt.c_str(), 0, 1, "down")) {
        DEBUG_PRINTLN(" ok");
        // ... and subscribe to topic
        client.subscribe(lSubscribe.c_str());
        DEBUG1_PRINTLN(lSubscribe);
        mqttConnected = true;
        //timer1.deactivate();
        mqttSet(para.mLwt, "up");
        if (inSetup){
          mqttSet("ResetReason", ESP.getResetReason());
          mqttSet("ResetInfo", ESP.getResetInfo());
          mqttSet("Heap", String(ESP.getFreeHeap()));
          mqttSet("ChipId", String(ESP.getChipId()));
          mqttSet("Version", mVersionNr+mVersionBoard);
        }
      }
    }
  }
}
