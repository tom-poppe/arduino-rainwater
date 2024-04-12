/*  
 *  Compile time options: 
 *
 *    - local settings/secrets file
 *
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

/* Globals */
byte wifiMacAddress[6];

/*
 *  STARTUP sequence:
 *
 *    - Setup communication
 *    - Connect to WIFI
 *    - Connect to MQTT
 *    - Setup devices
 *
 */

void setup()
{
  setupSerialMonitor();

  Serial.println("Starting rainwater monitor");

  setupWifi();
 
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