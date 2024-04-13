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

/*
 *  MAIN LOOP
 *
 *  - Read pit 1 & send MQTT
 *  - Read pit 2 & send MQTT
 *  - Read Water pressure & send MQTT
 *
 */

void loop()
{
  /* Read pit 1*/
  #ifdef USE_PIT_1
    float distance1 = readDistanceFromPit1();

    #ifdef USE_MQTT
      sendMqttMessage(PIT_1_MQTT_TOPIC, distance1);
    #endif
  #endif

  #ifdef USE_PIT_2
    float distance2 = readDistanceFromPit2();

    #ifdef USE_MQTT
      sendMqttMessage(PIT_2_MQTT_TOPIC, distance2);
    #endif
  #endif

  #ifdef USE_PRESSURE
    float pressureAnalog = readWaterPressure();
    float pressureVolt   = convertAnalogPressureToVolt(pressureAnalog);
    float pressureBar    = convertVoltPressureToKpa(pressureVolt);
    float pressurekPa    = convertVoltPressureToBar(pressureVolt);

    #ifdef USE_MQTT
      sendMqttMessage(MQTT_TOPIC_PRESSURE_ANALOG, pressureAnalog);
      sendMqttMessage(MQTT_TOPIC_PRESSURE_VOLT,   pressureVolt);
      sendMqttMessage(MQTT_TOPIC_PRESSURE_KPA,    pressureBar);
      sendMqttMessage(MQTT_TOPIC_PRESSURE_BAR,    pressurekPa);
    #endif
  #endif

  delay(ALARM_LOOP_TIME);
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

/* Read distance from sensor */

#ifdef USE_PIT_1

float readDistanceFromPit1()
{
  Serial.print("Getting a distance reading from ");
  Serial.println(PIT_1_NAME);

  unsigned char data[4]={};

  do {
    for (int i=0 ; i<4 ; i++) {
      data[i]=serialPit1.read();
    }
  } while(serialPit1.read()==0xff);

  serialPit1.flush();

  return parsedistance(PIT_1_NAME, data);
}

#endif

#ifdef USE_PIT_2

float readDistanceFromPit2()
{
  Serial.print("Getting a distance reading from ");
  Serial.println(PIT_2_NAME);

  unsigned char data[4]={};

  do {
    for (int i=0 ; i<4 ; i++) {
      data[i]=serialPit2.read();
    }
  } while(serialPit2.read()==0xff);

  serialPit2.flush();

  return parsedistance(PIT_2_NAME, data);
}

#endif

/* Check distance reading & convert digital to cm */

float parsedistance(String name, unsigned char data[4])
{
  float distance = 0.0;

  Serial.print("distance for ");
  Serial.print(name);
  Serial.print(" is ");

  if (data[0] == 0xff) {
    int sum = (data[0]+data[1]+data[2])&0x00FF;
    
    if (sum == data[3]) {
      distance = ((data[1]<<8)+data[2])/10;

      if (distance > 280) {
        Serial.print(distance);
        Serial.println("cm");

        return distance;
      } else {
        Serial.println("below the lower limit");
      }
    } else {
      Serial.println("faulty (checksum mismatch)");
    }
  } else {
    Serial.println("faulty (header mismatch)");
  }

  return NULL;
}

/* Read water pressure from analog sensor */

#ifdef USE_PRESSURE

float readWaterPressure()
{
  // Returns 0 (0.5V) to 1023 (4.5V)
  return analogRead(PRESSURE_READING_PIN);
}

float convertAnalogPressureToVolt(float analog)
{
  return analog * 5.00 / 1024;
}

float convertVoltPressureToKpa(float volt)
{
  return (volt - PRESSURE_CALIBRATION_OFFSET) * 250;
}

float convertVoltPressureToBar(float volt)
{
  return (volt - PRESSURE_CALIBRATION_OFFSET) * 250 / 100;
}

#endif /* USE_PRESSURE */

/* Send an MQTT message */

#ifdef USE_MQTT

void sendMqttMessage(String topic, float measurement)
{
  int qos = 1;
  bool dup = false;
  bool retain = true;

  if (measurement != NULL) {
    String payload = String(measurement);

    Serial.print("MQTT: sending ");
    Serial.print(topic);
    Serial.print(" with payload ");
    Serial.println(payload);

    mqttClient.beginMessage(topic, payload.length(), retain, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();
  } else {
    Serial.println("MQTT: not sending a NULL reading");
  }
}

#endif /* USE_MQTT */