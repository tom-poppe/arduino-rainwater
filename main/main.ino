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
 
}

/* Start serial monitor communication */

void setupSerialMonitor()
{
  Serial.begin(SERIAL_MONITOR_BAUD);

  while (!Serial) {
    ;
  }
}







void loop()
{
  delay(10000);
}