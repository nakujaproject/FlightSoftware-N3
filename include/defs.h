#ifndef DEFS_H
#define DEFS_H

/* debug enable for use during testing */
#define DEBUG 1

#if DEBUG == 1

#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x, y) Serial.printf(x, y)

#else

#define debug(x)
#define debugln(x)
#define debugf(x, y)

#endif

/* timing constant */
#define SETUP_DELAY 300
#define TASK_DELAY 10

/* flight constants */
#define EJECTION_HEIGHT 1000 // eject at 1000m AGL
#define SEA_LEVEL_PRESSURE 101325 // Assume the sea level pressure is 101325 Pascals - this can change with weather
#define ALTITUDE_OFFSET 1500 /* this value is the altitude at rocket launch site */

/* tasks constants */
#define STACK_SIZE 2048
#define ALTIMETER_QUEUE_LENGTH 10 // todo: change to 2 items
#define GYROSCOPE_QUEUE_LENGTH 10
#define FILTERED_DATA_QUEUE_LENGTH 10

/* MQTT constants */
const char* MQTT_SERVER = "192.168.1.100";
#define MQTT_BUFFER_SIZE 300
#define MQTT_PORT 1883

/* WIFI credentials */
const char* SSID = "onboard";
const char* PASSWORD = "123456789";

#endif