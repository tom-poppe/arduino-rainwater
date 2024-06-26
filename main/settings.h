/* General */

#define SERIAL_MONITOR_BAUD 9600
#define ALARM_LOOP_TIME 10000 /* 10 secs minimal loop time */

/* MQTT */

#define USE_MQTT true /* remove if not used */

#define MQTT_HOST "your mqtt host"
#define MQTT_PORT 1883

/* LCD */

#define USE_LCD true /* remove if not used */
#define LCD_I2C_ADDRESS 0x20 /* i2c */
#define LCD_LINES 2
#define LCD_COLS 16

/* PIT sensors */

#define BAUD_ULTRASONIC_SENSOR 9600
#define DISTANCE_MAX_RETRY_COUNT 10

/* PIT 1 distance */

#define USE_PIT_1 true /* remove if not used */

#define PIT_1_NAME "Name pit 1"
#define PIT_1_MQTT_TOPIC "sensor/state/pit1/distance"

/* PIT 2 distance */

#define USE_PIT_2 true /* remove if not used */

#define PIT_2_NAME "Name pit 2"
#define PIT_2_MQTT_TOPIC "sensor/state/pit2/distance"

/* Use water pressure reading */

#define USE_PRESSURE true /* remove if not used */

#define PRESSURE_READING_PIN 0
#define PRESSURE_CALIBRATION_OFFSET 0.483 /* See doc for sensor */
#define PRESSURE_MINIMUM_THRESHOLD 2.00 /* Do more readings when it drops below */
#define PRESSURE_READING_LOOP_TIME 60000  /* do 1 reading per minute */

#define MQTT_TOPIC_PRESSURE_ANALOG "sensor/state/rainwater_in_shed/pressure_analog"
#define MQTT_TOPIC_PRESSURE_VOLT   "sensor/state/rainwater_in_shed/pressure_volt"
#define MQTT_TOPIC_PRESSURE_KPA    "sensor/state/rainwater_in_shed/pressure_kpa"
#define MQTT_TOPIC_PRESSURE_BAR    "sensor/state/rainwater_in_shed/pressure_bar"