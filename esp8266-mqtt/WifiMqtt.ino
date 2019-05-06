void wifiStaSetup() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    int i = 0;
    DEBUG_PRINT("WiFi... ");
    WiFi.begin(para.ssid, para.password);
    while (WiFi.status() != WL_CONNECTED && i++ < 10) {
      delay(500);
      DEBUG_PRINT(".");
    }
    if (i >= 10){
      wifiAPSetup();
    } else {
      DEBUG_PRINT(", IP: ");
      DEBUG_PRINT(WiFi.localIP());
      DEBUG_PRINT(", MAC: ");
      WiFi.macAddress(para.mMac);                   //get MAC address of interface
  
      for (i = 0; i < 6; ++i) {
        if (i > 0) {
          DEBUG_PRINT(":");
        }
        if (para.mMac[i] < 0x10) {
          DEBUG_PRINT("0");
        }
        DEBUG_PRINT(para.mMac[i], HEX);
      }
      DEBUG_PRINT(", Client: ");
      DEBUG_PRINT(para.mClient);
      DEBUG_PRINTLN(" ok");
      wifiStation = true;
    }
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
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "esp-" + macID;
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
  DEBUG_PRINT("failed, rc=");
  DEBUG_PRINT(client.state());
  DEBUG_PRINT(", ");
  DEBUG_PRINT(para.mqtt_server);
  DEBUG_PRINT(", ");
  DEBUG_PRINTLN(para.mqtt_port);
}

void mqttReconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    // MQTT disconnected
    mqttFailed();
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi disconnected
      DEBUG_PRINT("WiFi.status()...");
      DEBUG_PRINTLN(WiFi.status());
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
        //timer1.deactivate();
        mqttSet(para.mLwt, "up");
        mqttSet("ResetReason", ESP.getResetReason());
        mqttSet("ResetInfo", ESP.getResetInfo());
        mqttSet("Heap", String(ESP.getFreeHeap()));
        mqttSet("ChipId", String(ESP.getChipId()));
        mqttSet("Version", mVersionNr+mVersionBoard);
        mqttSet("IP", WiFi.localIP().toString());
        //DEBUG_PRINTLN(ESP.getChipId());
      } else {
        mqttFailed();      
      }
    }
  }
}
