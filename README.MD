
# esp8266-mqtt
Das Projekt wird in einer **[FHEM](https://fhem.de/fhem_DE.html)**-Installation genutzt, um Sensoren abzufragen und Aktoren zu schalten. Eingesammelt und verteilt werden die Daten über das **MQTT**-Protokoll, installiert ist [mosquitto](https://mosquitto.org/).
## Wie werden die Chips genutzt
 - Der esp8266 wird als Wemos mini oder NodeMcu genutzt und mit micro-USB-Adapter mit Strom versorgt.
 - **I2C** und **1-Wire** werden beim Start geprüft und eingebunden. 
 - Enthalten sind die Bibliotheken für Helligkeit (BH1750FVI); Luftdruck,Luftfeuchte und Temperatur (BMP280); Temperatur (DS1820); Zwei **digitale** und ein **analoger** Port sind als Eingang geschaltet.
 - Ohne WLan-Zugangsdaten wird ein Webserver unter http://192.168.4.1 aktiviert, in dem die Software konfiguriert werden kann. 
 - Mit gültigen WLan-Zugangsdaten (WPA2) loggt sich der Chip ein und verbindet sich mit dem MQTT-Server.
 - **Over-the-air-Updates** sind möglich, wenn auf dem MQTT-Server auch ein Webserver mit PHP-Unterstützung installiert ist.
## Bibliotheken
Folgende Bibliotheken sind eingebunden, aber nicht über dieses Repository herunterladbar: 
 **ESP8266WiFi** für die Grundfunktionalität WLan
 **ESP8266httpUpdate** // enables OTA updates
 **ESP8266WebServer** // enables webconfig
 **ESP8266mDNS**
 **FS** // enables storing webpages in EEPROM, not in sketch
 **EEPROM** // enables storing config-data in EEPROM
 **WiFiUdp** // enables NTP / RTC

 **PubSubClient**  // MQTT, [knolleary/pubsubclient](https://github.com/knolleary/pubsubclient), MIT-Lizenz
 **Wire** // serial monitor
 **BMP280** // IC BMP280
 **BH1750FVI** // IC BH1750
 **OneWire** // 1-wire
 **DallasTemperature** // 1-wire temperature

# Lizenz
Lizensiert unter GNU General Public License v3.0-or-later
Copyright © 2017 Manfred Bielemeier 
