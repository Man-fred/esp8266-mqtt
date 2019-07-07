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
#endif BLE
