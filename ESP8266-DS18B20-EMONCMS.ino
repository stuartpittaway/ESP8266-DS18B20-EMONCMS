/*
  _____ ___________ _____  _____   ____  ____
  |  ___/  ___| ___ \  _  |/ __  \ / ___|/ ___|
  | |__ \ `--.| |_/ /\ V / `' / /'/ /___/ /___
  |  __| `--. \  __/ / _ \   / /  | ___ \ ___ \
  | |___/\__/ / |   | |_| |./ /___| \_/ | \_/ |
  \____/\____/\_|   \_____/\_____/\_____|_____/

  ______  _____ __  ___________  _____  _____
  |  _  \/  ___/  ||  _  | ___ \/ __  \|  _  |
  | | | |\ `--.`| | \ V /| |_/ /`' / /'| |/' |
  | | | | `--. \| | / _ \| ___ \  / /  |  /| |
  | |/ / /\__/ /| || |_| | |_/ /./ /___\ |_/ /
  |___/  \____/\___|_____|____/ \_____/ \___/



  (c) 2017 Stuart Pittaway

  This code runs on ESP-8266-12E (NODE MCU 1.0) and compiles with Arduino 1.8.5 environment

  It is designed to detect DS18B20 temperature sensors/probes connected to the ESP8266, regularly read these
  and transmit the reading to Open Energy Monitor (emoncms)

  This code is provided as-is without warranty of any kind.

  Note that there are likely to be buffer overflow vulnerabilities on the web form submission - no checks are done on the validity of the data

  Setting up ESP-8266-12E (NODE MCU 1.0) on Arduino
  http://www.instructables.com/id/Programming-a-HTTP-Server-on-ESP-8266-12E/

  Arduino program settings:
  NodeMCU 1.0 (ESP-8266-12E module), Flash 4M (3MSPIFF), CPU 80MHZ

  Hookup guide:
  - DS18B20:
     + connect VCC (3.3V) to the appropriate DS18B20 pin (VDD)
     + connect GND to the appopriate DS18B20 pin (GND)
     + connect D6 to the DS18B20 data pin (DQ)
     + connect a 4.7K resistor between DQ and VCC

  - Pull PIN D5 to GND and power on/reset to force factory reset

*/

extern "C"
{
  //This is part of the Ardunio ESP8266 library
#include "user_interface.h"
}


//30 second interval
#define INTERVAL_BETWEEN_READINGS_MS 30000

#define TEMPERATURE_PRECISION 10
#define MAXIMUM_NUM_SENSORS 16

//Pin to find i2c devices
#define ONE_WIRE_BUS            D6      // DS18B20 pin

// LED Pin
#define LED_PIN D4

// Switch PIN (pull to ground to switch on)
#define FACTORY_RESET_PIN D5

#define LED_ON digitalWrite(LED_PIN, LOW)
#define LED_OFF digitalWrite(LED_PIN, HIGH)

#define EEPROM_storageSize 1024
#define EEPROM_CHECKSUM_ADDRESS 0
#define EEPROM_CONFIG_ADDRESS EEPROM_CHECKSUM_ADDRESS+sizeof(uint32_t)

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// https://arduinojson.org
#include <ArduinoJson.h>

ESP8266WebServer server(80);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress deviceAddress[MAXIMUM_NUM_SENSORS];
float lastReading[MAXIMUM_NUM_SENSORS];

uint8_t ds18Count = 0; // Number of DS18xxx Family devices
os_timer_t myTimer;

//We have allowed space for 1024-256 bytes of EEPROM for settings (768 bytes)
struct eeprom_settings {
  char wifi_ssid[32 + 1];
  char wifi_passphrase[63 + 1];

  bool emoncms_enabled;
  uint8_t emoncms_node;
  int emoncms_httpPort;
  char emoncms_host[64 + 1];
  char emoncms_apikey[32 + 1];
  char emoncms_url[64 + 1];
};
eeprom_settings myConfig;

const char* ssid = "ESP8266_TEMP_MON";

String networks;

void handleNotFound()
{
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}

void sendHeaders()
{
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "private");
}

String htmlHeader() {
  return String(F("<!DOCTYPE HTML>\r\n<html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>.page {width:95%;margin:0 0 0 0;background-color:cornsilk;font-family:sans-serif;padding:12px;} label {min-width:80px;display:inline-block;padding:6px 0 6px 0;}</style></head><body><div class=\"page\"><h1>ESP8266 TEMPERATURE MONITOR</h1>"));
}

String htmlFooter() {
  return String(F("</div></body></html>\r\n\r\n"));
}


void handleRoot()
{
  String s;
  s = htmlHeader();
  //F Macro - http://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html
  s += F("<h2>WiFi Setup</h2><p>Select local WIFI to connect to:</p><form autocomplete=\"off\" id=\"form_emoncms\" method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"\\save\"><label for=\"ssid\">SSID:</label><select id=\"ssid\" name=\"ssid\">");
  s += networks;
  s += F("</select>");
  s += F("<div><label for=\"pass\">Password:</label><input type=\"password\" id=\"id\" name=\"pass\"></div>");
  s += F("<h3>emonCMS connection details</h3>");
  s += "<div><label for=\"emoncms_enabled\">Enabled:</label><input id=\"emoncms_enabled\" name=\"emoncms_enabled\" type=\"checkbox\" value=\"1\" " + String(myConfig.emoncms_enabled ? "checked=\"\"" : "") + "  /></div>";
  s += "<div><label for=\"emoncms_host\">Host:</label><input id=\"emoncms_host\" name=\"emoncms_host\" size=\"32\" type=\"text\" value=\"" + String(myConfig.emoncms_host) + "\"/></div>";
  s += "<div><label for=\"emoncms_httpPort\">HTTP Port:</label><input min=\"1\" max=\"65535\" id=\"emoncms_httpPort\" name=\"emoncms_httpPort\" size=\"40\" type=\"number\"  value=\"" + String(myConfig.emoncms_httpPort) + "\"/></div>";
  s += "<div><label for=\"emoncms_node\">Node number:</label><input min=\"1\" max=\"1024\" id=\"emoncms_node\" name=\"emoncms_node\" size=\"40\" type=\"number\"  value=\"" + String(myConfig.emoncms_node) + "\"/></div>";
  s += "<div><label for=\"emoncms_url\">URI:</label><input id=\"emoncms_url\" name=\"emoncms_url\" size=\"32\" type=\"text\"  value=\"" + String(myConfig.emoncms_url) + "\"/></div>";
  s += "<div><label for=\"emoncms_apikey\">API key:</label><input id=\"emoncms_apikey\" type=\"password\" name=\"emoncms_apikey\" size=\"32\" type=\"text\" value=\"" + String(myConfig.emoncms_apikey) + "\"/></div>";
  s += F("<input  type=\"submit\" value=\"Submit\"></form>");
  s += htmlFooter();

  sendHeaders();
  server.send(200, "text/html", s);
}

void FactoryResetSettings() {
  const char emoncms_host[] = "192.168.0.26";
  const char emoncms_apikey[] = "1234567890abcdef1234567890abcdef";
  const char emoncms_url[] = "/emoncms/input/post";

  strcpy(myConfig.emoncms_host, emoncms_host );
  strcpy(myConfig.emoncms_apikey, emoncms_apikey);
  strcpy(myConfig.emoncms_url, emoncms_url);

  myConfig.emoncms_enabled = false;
  myConfig.emoncms_node = 35;
  myConfig.emoncms_httpPort = 80;
}


uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  //This calculates a CRC32 the same as used in MPEG2 streams
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void WriteConfigToEEPROM() {
  uint32_t checksum = calculateCRC32((uint8_t*)&myConfig, sizeof(eeprom_settings));
  EEPROM.begin(EEPROM_storageSize);
  EEPROM.put(EEPROM_CONFIG_ADDRESS, myConfig);
  EEPROM.put(EEPROM_CHECKSUM_ADDRESS, checksum);
  EEPROM.end();
}


bool LoadConfigFromEEPROM() {
  eeprom_settings restoredConfig;
  uint32_t existingChecksum;

  EEPROM.begin(EEPROM_storageSize);
  EEPROM.get(EEPROM_CONFIG_ADDRESS, restoredConfig);
  EEPROM.get(EEPROM_CHECKSUM_ADDRESS, existingChecksum);
  EEPROM.end();

  // Calculate the checksum of an entire buffer at once.
  uint32_t checksum = calculateCRC32((uint8_t*)&restoredConfig, sizeof(eeprom_settings));

  if (checksum == existingChecksum) {
    //Clone the config into our global variable and return all OK
    memcpy(&myConfig, &restoredConfig, sizeof(eeprom_settings));
    return true;
  }

  //Config is not configured or gone bad, return FALSE
  return false;
}

void handleSave() {
  String s;
  String ssid = server.arg("ssid");
  String password = server.arg("pass");

  if ((ssid.length() <= sizeof(myConfig.wifi_ssid)) && (password.length() <= sizeof(myConfig.wifi_passphrase))) {

    memset(&myConfig, 0, sizeof(eeprom_settings));

    ssid.toCharArray(myConfig.wifi_ssid, sizeof(myConfig.wifi_ssid));
    password.toCharArray(myConfig.wifi_passphrase, sizeof(myConfig.wifi_passphrase));

    myConfig.emoncms_enabled = (server.arg("emoncms_enabled").toInt() == 1) ? true : false;
    myConfig.emoncms_node = server.arg("emoncms_node").toInt();
    myConfig.emoncms_httpPort = server.arg("emoncms_httpPort").toInt();

    server.arg("emoncms_host").toCharArray(myConfig.emoncms_host, sizeof(myConfig.emoncms_host));
    server.arg("emoncms_url").toCharArray(myConfig.emoncms_url, sizeof(myConfig.emoncms_url));
    server.arg("emoncms_apikey").toCharArray(myConfig.emoncms_apikey, sizeof(myConfig.emoncms_apikey));

    WriteConfigToEEPROM();

    s = htmlHeader() + F("<p>Settings saved, please reboot ESP8266 device</p>") + htmlFooter();
    sendHeaders();
    server.send(200, "text/html", s);

    while (1) {
      delay(250);
      yield();
    }
    //ESP.restart();

  } else {
    s = htmlHeader() + F("<p>WIFI settings too long.</p>") + htmlFooter();
    sendHeaders();
    server.send(200, "text/html", s);
  }
}

void setupAccessPoint(void) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);
  int n = WiFi.scanNetworks();

  if (n == 0)
    networks = "no networks found";
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (WiFi.encryptionType(i) != ENC_TYPE_NONE) {
        // Only show encrypted networks
        networks += "<option>";
        networks += WiFi.SSID(i);
        networks += "</option>";
      }
      delay(10);
    }
  }

  /* Soft AP network parameters */
  WiFi.mode(WIFI_AP);

  IPAddress apIP(192, 168, 4, 1);
  IPAddress netMsk(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(ssid);
  delay(500); // Without delay I've seen the IP address blank
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());



  if (!MDNS.begin("esp8266-temp-mon")) {
    Serial.println("Error setting up MDNS responder!");
    //This will force a reboot of the ESP module by hanging the loop
    while (1) {
      delay(1000);
    }
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleNotFound);

  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.println("Soft AP ready");
  while (1) {
    server.handleClient();
  }
}

void handleSupplyReadingsJSON() {
  //You can call this web page "/json" from a browser to return the JSON value of the latest readings
  String s="";

  //May need to increase this for large data sets with lots of temperature readings
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();

  for (int i = 0; i < ds18Count; i++ ) {   
    float t=lastReading[i]; 
    if (t != DEVICE_DISCONNECTED_C && t != DEVICE_DISCONNECTED_RAW) {
      data[GetDeviceAddressAsString(deviceAddress[i])] = t;
    } else {
      //Error - null value
      data[GetDeviceAddressAsString(deviceAddress[i])] = (char*)0; // or (char*)NULL if you prefer
    }
  }

  data.printTo(s);

  //Release memory
  jsonBuffer.clear();

  sendHeaders();
  server.send(200, "application/json", s);
}


String GetDeviceAddressAsString(DeviceAddress a) {

  String address = "";
  //TODO: THERE MUST BE A BETTER WAY THAN THIS!
  for (uint8_t x = 0; x < 8; x++)
  {
    // zero pad the address if necessary
    if (a[x] < 16) address += "0";

    address += String(a[x], HEX);
  }

  return address;
}



void handleSupplyReadings() {
  String s;
  s = htmlHeader();
  s += F("<table border=1><tr><th>Chip serial number</th><th>Reading (oC)</th></tr>");

  for (int i = 0; i < ds18Count; i++ ) {

    s += "<tr><td>";

    s += GetDeviceAddressAsString(deviceAddress[i]);

    s += "</td><td>" + String(lastReading[i]) + "</td>";

    s += "</tr>";
  }

  s += "</table>";
  s += htmlFooter();
  sendHeaders();
  server.send(200, "text/html", s);
}



volatile bool readyToTransmit = false;

String jsonPayload = "";


void timerCallback(void *pArg) {
  LED_ON;

  //Only run if there are sensors and we have an empty buffer
  if (ds18Count > 0 && readyToTransmit == false) {

    //May need to increase this for large data sets with lots of temperature readings
    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& data = jsonBuffer.createObject();

    sensors.requestTemperatures(); // Send the command to get temperatures

    for (int i = 0; i < ds18Count; i++ ) {
      float t = sensors.getTempC(deviceAddress[i]);

      lastReading[i] = t;

      //Only output for sensors still connected...
      if (t != DEVICE_DISCONNECTED_C && t != DEVICE_DISCONNECTED_RAW) {
        //Create the JSON object
        data[GetDeviceAddressAsString(deviceAddress[i])] = t;
      }
    }

    jsonPayload = "";
    data.printTo(jsonPayload);

    //Release memory
    jsonBuffer.clear();

    readyToTransmit = true;
  }

  LED_OFF;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(FACTORY_RESET_PIN, INPUT);

  Serial.begin(19200);           // start serial for output
  Serial.println();
  //Serial.setDebugOutput(true);

  Serial.println(F("ESP8266 DS18B20 temperature module"));

  bool validSettings = LoadConfigFromEEPROM();

  if (!validSettings) {
    FactoryResetSettings();
  }

  //If factory reset pin is pulled LOW - force reset of settings
  if (digitalRead(FACTORY_RESET_PIN) == LOW) {
    validSettings = false;
  }

  if (validSettings) {
    Serial.println(F("Connect to WIFI AP"));
    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
      would try to act as both a client and an access-point and could cause
      network-issues with your other WiFi-devices on your WiFi-network. */
    WiFi.mode(WIFI_STA);
    WiFi.begin(myConfig.wifi_ssid, myConfig.wifi_passphrase);
  } else {
    Serial.println("We are in initial power on mode - starting WIFI access point");
    setupAccessPoint();
  }

  //Scan for DS18 modules...
  oneWire.reset_search();

  DeviceAddress scanAddress;
  while (oneWire.search(scanAddress) && ds18Count < MAXIMUM_NUM_SENSORS) {

    if (sensors.validAddress(scanAddress)) {
      if (sensors.validFamily(scanAddress)) {
        memcpy(&deviceAddress[ds18Count], &scanAddress, sizeof(scanAddress));
        sensors.setResolution(deviceAddress[ds18Count], TEMPERATURE_PRECISION, false);

        Serial.println(GetDeviceAddressAsString(deviceAddress[ds18Count]));


        Serial.println();
        ds18Count++;
      }
    }
  }

  Serial.print("Found ");
  Serial.print(ds18Count);
  Serial.println(" temp sensors.");


  //TODO: We need a timeout here in case the AP is dead!
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print( WiFi.status() );
  }
  Serial.print(F(". Connected IP:"));
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleSupplyReadings);
  server.on("/json", HTTP_GET, handleSupplyReadingsJSON);
  server.onNotFound(handleNotFound);

  server.begin();
  MDNS.addService("http", "tcp", 80);

  //Ensure we service the cell modules every 30 seconds
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, INTERVAL_BETWEEN_READINGS_MS, true);

  LED_OFF;

  Serial.println("setup finished");
}


void loop() {
  //Handle any wifi events
  server.handleClient();
  yield();
  delay(150);
  server.handleClient();
  yield();
  delay(150);
  server.handleClient();


  if (readyToTransmit && myConfig.emoncms_enabled  && (WiFi.status() == WL_CONNECTED)) {

    Serial.println(F("About to upload to emoncms..."));

    WiFiClient client;

    if (!client.connect(myConfig.emoncms_host, myConfig.emoncms_httpPort)) {
      Serial.println("connection failed");

    } else {


      //Debug
      Serial.println(jsonPayload);

      //NOTE YOU CANNOT PUT WIFI COMMANDS INSIDE THE TIMER CALLBACK...
      // This will send the request to the server
      client.print(String("GET ")
                   + String(myConfig.emoncms_url)
                   + "?node="
                   + String(myConfig.emoncms_node)
                   + "&fulljson=" + jsonPayload
                   + " HTTP/1.1\r\n"
                   + "Host: " + myConfig.emoncms_host
                   + "\r\nConnection: close\r\nAuthorization: Bearer " + String(myConfig.emoncms_apikey)
                   + "\r\n\r\n");

      //Expect reply within 2.5 seconds
      unsigned long timeout = millis() + 2500;

      // Read all the lines of the reply from server and print them to Serial
      while (client.connected())
      {
        yield();

        if (millis() > timeout) {
          Serial.println(">>> Client Timeout !");
          client.stop();
          return;
        }

        if (client.available())
        {
          String line = client.readStringUntil('\n');
          Serial.println(line);
        }
      }
      client.stop();
    }

    readyToTransmit = false;
  }
}//end loop
