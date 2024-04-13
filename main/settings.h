#define SERIAL_MONITOR_BAUD 9600

#define USE_MQTT true /* remove if not used */

#define MQTT_HOST "your mqtt host"
#define MQTT_PORT 1883

#define USE_PIT_1 true /* remove if not used */

#define PIT_1_NAME "Name pit 1"
#define PIT_1_RX_PIN 11
#define PIT_1_TX_PIN 10
#define PIT_1_MQTT_TOPIC "sensor/state/pit1/distance"

#define USE_PIT_2 true /* remove if not used */

#define PIT_2_NAME "Name pit 2"
#define PIT_2_RX_PIN 9
#define PIT_2_TX_PIN 8
#define PIT_2_MQTT_TOPIC "sensor/state/pit2/distance"

#define BAUD_ULTRASONIC_SENSOR 57600

#define ALARM_LOOP_TIME 10000 /* 10 secs minimal loop time */