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
#include <SPI.h>
#include <WiFi.h>

#ifdef USE_MQTT
  #include <ArduinoMqttClient.h>
#endif

#ifdef USE_LCD
  #include <LiquidCrystal_I2C.h>
#endif

/* Globals */
byte wifiMacAddress[6];
unsigned long lastPressureReadingTime = 0;
float lastPressureReadingBar = 0;
unsigned char distanceData[4]={};

#ifdef USE_MQTT 
  WiFiClient wifiClient;
  MqttClient mqttClient(wifiClient);
#endif 

#ifdef USE_LCD
  LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_LINES);
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
  #ifdef USE_LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
  #endif

  setupSerialMonitor();

  Serial.println("Starting rainwater monitor");
  lcdPrint(0, 0, "Starting rain",0);
  lcdPrint(1, 0, "water monitor",0);

  setupWifi();

  #ifdef USE_MQTT
    setupMqtt();
  #endif

  #ifdef USE_PIT_1
    Serial.print("Setting up serial comms for ultrasonic sensor for ");
    Serial.print(PIT_1_NAME);
    Serial.print(" to baud ");
    Serial.println(BAUD_ULTRASONIC_SENSOR);

    Serial1.begin(BAUD_ULTRASONIC_SENSOR);

    lcdClear();
    lcdPrint(0,0,"Setting BAUD",0);
    lcdPrint(1,0,PIT_1_NAME,0);
  #endif

  #ifdef USE_PIT_2
    Serial.print("Setting up serial comms for ultrasonic sensor for ");
    Serial.print(PIT_2_NAME);
    Serial.print(" to baud ");
    Serial.println(BAUD_ULTRASONIC_SENSOR);

    Serial2.begin(BAUD_ULTRASONIC_SENSOR);

    lcdClear();
    lcdPrint(0,0,"Setting BAUD",0);
    lcdPrint(1,0,PIT_2_NAME,0);
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
  lcdClear();
  lcdPrint(0,0,"Setting up WIFI",0);

  initWifiModule();
  connectToWifiNetwork();
  printNetworkStats();
}

void initWifiModule()
{
  Serial.println("Initializing WIFI module");
  lcdPrint(1,0,"Init WIFI mod",0);

  if (WiFi.status() == WL_NO_MODULE) {
    die("Communication with WiFi module failed!");
  }

  WiFi.macAddress(wifiMacAddress);
}

void connectToWifiNetwork()
{
  int wifiStatus = WL_IDLE_STATUS;

  while (wifiStatus != WL_CONNECTED) {
    lcdClear();
    lcdPrint(0,0,"Connecting to",0);
    lcdPrint(1,0,WIFI_SSID,0);

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

  lcdClear();
  lcdPrint(0,0,"Connecting MQTT",0);
  lcdPrint(1,0,MQTT_HOST,0);

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
 *  TODO: output to LCD?
 */

void loop()
{
  float distance;

  /* Show network */
  printNetworkStats();

  /* Read pit 1*/

  #ifdef USE_PIT_1
    distance = readDistanceFromPit(1);

    lcdClear();
    lcdPrint(0,0,"Distance ",0);
    lcdPrint(0,9,PIT_1_NAME,0);
    lcdPrint(1,0,String(distance), 2000);

    #ifdef USE_MQTT
      sendMqttMessage(PIT_1_MQTT_TOPIC, distance);
    #endif
  #endif

  #ifdef USE_PIT_2
    distance = readDistanceFromPit(2);

    lcdClear();
    lcdPrint(0,0,"Distance ",0);
    lcdPrint(0,9,PIT_2_NAME,0);
    lcdPrint(1,0,String(distance), 2000);

    #ifdef USE_MQTT
      sendMqttMessage(PIT_2_MQTT_TOPIC, distance);
    #endif
  #endif

  #ifdef USE_PRESSURE
    /* TODO: is there a way of knowing whether measurement is succesfull/something is connected? */
    /* https://arduino.stackexchange.com/questions/84728/can-i-test-if-something-is-connected-to-analog-pin */
    /* Also, are there better ways to read ultrasonic serial? */

    if (
      ((millis() - lastPressureReadingTime) > PRESSURE_READING_LOOP_TIME) ||
      (lastPressureReadingBar < PRESSURE_MINIMUM_THRESHOLD)
    ) { /* Less readings = longer lifespan? */
      float pressureAnalog = readWaterPressure();
      float pressureVolt   = convertAnalogPressureToVolt(pressureAnalog);
      float pressureBar    = convertVoltPressureToKpa(pressureVolt);
      float pressurekPa    = convertVoltPressureToBar(pressureVolt);

      lcdClear();
      lcdPrint(0,0,"Pressure",0);
      lcdPrint(1,0,String(pressureBar), 2000);

      #ifdef USE_MQTT
        sendMqttMessage(MQTT_TOPIC_PRESSURE_ANALOG, pressureAnalog);
        sendMqttMessage(MQTT_TOPIC_PRESSURE_VOLT,   pressureVolt);
        sendMqttMessage(MQTT_TOPIC_PRESSURE_KPA,    pressureBar);
        sendMqttMessage(MQTT_TOPIC_PRESSURE_BAR,    pressurekPa);
      #endif

      lastPressureReadingTime = millis();
      lastPressureReadingBar = pressureBar;
    }
  #endif

  delay(ALARM_LOOP_TIME);
}


/* Print a message & stop */

void die(String error)
{
  while(true) {
    lcdClear();
    lcdPrint(0,0,"PANIC!",0);
    lcdPrint(1,0,error,0);

    Serial.print("Panic: ");
    Serial.println(error);
    delay(10000);
  }
}

/* Print network statistics */

void printNetworkStats()
{
  Serial.println("WIFI settings:");
  lcdClear();

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
  /* TODO: print MAC to LCD */
  for (int i = 5; i >= 0; i--) {
    if (wifiMacAddress[i] < 16) {
      Serial.print("0");
    }

    Serial.print(wifiMacAddress[i], HEX);

    if (i > 0) {
      Serial.print(":");
    }
  }

  Serial.println();
}

/* Print WIFI SSID */

void printWifiSsid()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  lcdPrint(0,0,WiFi.SSID(),0);
}

/* Print WIFI IP */

void printWifiIp()
{
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  lcdPrint(1,0,WiFi.localIP().toString(),0);
}

/* Read distance from sensor */

#if defined(USE_PIT_1) || defined(USE_PIT_2)

float readDistanceFromPit(int pitNumber)
{
  Serial.print("Getting a distance reading from ");

  if (pitNumber == 1) {
    Serial.println(PIT_1_NAME);
  } else if (pitNumber == 2) {
    Serial.println(PIT_2_NAME);
  }

  return getReadingFromPit(pitNumber);
}

/* Try to get a correct reading*/

float getReadingFromPit(int pitNumber)
{
  for (int loop = 0 ; loop < DISTANCE_MAX_RETRY_COUNT ; loop++) { // 60 Tries to get a good reading
    #ifdef USE_PIT_1
      if (pitNumber == 1) {
        do {
          for (int i=0 ; i<4 ; i++) {
            distanceData[i] = Serial1.read();
          }
        } while (Serial1.read() == 0xff);

        Serial1.flush();
      }
    #endif /* USE_PIT_1 */

    #ifdef USE_PIT_2
      if (pitNumber == 2) {
        do {
          for (int i=0 ; i<4 ; i++) {
            distanceData[i] = Serial2.read();
          }
        } while (Serial.read() == 0xff);

        Serial2.flush();
      }
    #endif /* USE_PIT_2 */

    if (distanceData[0] == 0xff) {
      int sum;
      sum = (distanceData[0] + distanceData[1] + distanceData[2]) & 0x00FF;
    
      if(sum == distanceData[3]) {
        float distance = (distanceData[1]<<8) + distanceData[2];
        
        if(distance>280) {
          Serial.print("Measured ");
          Serial.print(distance);
          Serial.println("mm");

          return distance;
        } else {
          Serial.println("Below the lower limit (280mm)");
        }
      } else {
        Serial.println("Wrong checksum for reading, doing retry");
      }
    } else {
      Serial.println("Wrong header for reading, doing retry");
    }

    delay(500);
  }

  Serial.println("Unable to get a good reading");

  return NULL; /* No reading */
}

#endif /* USE_PITx */

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

void lcdClear()
{
  #ifdef USE_LCD
    lcd.clear();
  #endif
}

void lcdPrint(int row, int col, String message, int duration)
{
  #ifdef USE_LCD
    lcd.setCursor(col,row);
    lcd.print(message);

    delay(duration);
  #endif
}