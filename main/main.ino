/*  
 *  Compile time options: 
 *
 *    - local settings/secrets file
 *    - mqtt
 *    - pits
 */

/* Copy secrets.h to secrets.local.h and adjust your values */
#pragma once

#if __has_include("secrets.local.h")
  #include "secrets.local.h"
#else
  #include "secrets.h"
#endif

/* Copy settings.h to settings.local.h and adjust your values */
#pragma once

#if __has_include("settings.local.h")
  #include "settings.local.h"
#else
  #include "settings.h"
#endif

/* Libs */
#include <WiFiS3.h>

#ifdef USE_MQTT
  #include <ArduinoMqttClient.h>
#endif

#ifdef USE_PIT_1
  #include <SoftwareSerial.h>
#endif

/* Globals */
byte wifiMacAddress[6];

#ifdef USE_MQTT
  WiFiClient wifiClient;
  MqttClient mqttClient(wifiClient);
#endif 

#ifdef USE_PIT_1
  SoftwareSerial serialPit1(PIT_1_RX_PIN, PIT_1_TX_PIN);
#endif

#ifdef USE_PIT_2
  SoftwareSerial serialPit2(PIT_2_RX_PIN, PIT_2_TX_PIN);
#endif

/*
 *  STARTUP sequence:
 *
 *    - Setup communication
 *    - Connect to WIFI
 *    - Connect to MQTT
 *    - Setup digital serials
 *
 */

void setup()
{
  setupSerialMonitor();

  Serial.println("Starting rainwater monitor");

  setupWifi();

  #ifdef USE_MQTT
    setupMqtt();
  #endif

  #ifdef USE_PIT_1
    Serial.print("Setting up serial comms for ultrasonic sensor for ");
    Serial.print(PIT_1_NAME);
    Serial.print(" to baud ");
    Serial.println(BAUD_ULTRASONIC_SENSOR);

    serialPit1.begin(BAUD_ULTRASONIC_SENSOR);
  #endif

  #ifdef USE_PIT_2
    Serial.print("Setting up serial comms for ultrasonic sensor for ");
    Serial.print(PIT_2_NAME);
    Serial.print(" to baud ");
    Serial.println(BAUD_ULTRASONIC_SENSOR);

    serialPit2.begin(BAUD_ULTRASONIC_SENSOR);
  #endif

  Serial.println("Startup sequence done");
}

/* Start serial monitor communication */

void setupSerialMonitor()
{
  Serial.begin(SERIAL_MONITOR_BAUD);

  while (!Serial) {
    ;
  }
}

/* Setup WIFI: init module & connect to network */

void setupWifi()
{
  Serial.println("Setting up WIFI");

  initWifiModule();
  connectToWifiNetwork();
  printNetworkStats();
}

void initWifiModule()
{
  Serial.println("Initializing WIFI module");

  if (WiFi.status() == WL_NO_MODULE) {
    die("Communication with WiFi module failed!");
  }

  WiFi.macAddress(wifiMacAddress);
}

void connectToWifiNetwork()
{
  int wifiStatus = WL_IDLE_STATUS;

  while (wifiStatus != WL_CONNECTED) {
    printWifiMacAddress();

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);

    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);

    delay(10000);
  }
}

/* MQTT functions: setup, send, ... */

#ifdef USE_MQTT

void setupMqtt()
{
  Serial.print("Connecting to MQTT broker ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setUsernamePassword(MQTT_LOGIN, MQTT_PASS);

  if (!mqttClient.connect(MQTT_HOST, MQTT_PORT)) {

    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    die("Failed to connect to MQTT broker");
  }
}

#endif /* USE_MQTT */

void loop()
{
  delay(10000);
}


/* Print a message & stop */

void die(String error)
{
  while(true) {
    Serial.print("Panic: ");
    Serial.println(error);
    delay(10000);
  }
}

/* Print network statistics */

void printNetworkStats()
{
  Serial.println("WIFI settings:");

  // Mac
  printWifiMacAddress();

  // SSID
  printWifiSsid();

  // IP
  printWifiIp();
}

/* Print WIFI Mac Address */

void printWifiMacAddress()
{
  Serial.print("Mac address: ");

  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      Serial.print(":");
    }
    if (wifiMacAddress[i] < 16) {
      Serial.print("0");
    }
    Serial.print(wifiMacAddress[i], HEX);
  }
  Serial.println();
}

/* Print WIFI SSID */

void printWifiSsid()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
}

/* Print WIFI IP */

void printWifiIp()
{
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}