#ifndef DEFS_H
#define DEFS_H

/* debug parameters for use during testing - disable before launch */
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

#define DISPLAY_DATA_DEBUG 1 // if enabled, data will be displayed on the serial monitor
#define FSM_COUNTER_DEBUG 1

/* end of debug parameters */

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
#define GPS_QUEUE_LENGTH 24
#define ALL_TELEMETRY_DATA_QUEUE_LENGTH  10
#define FILTERED_DATA_QUEUE_LENGTH 10
#define FLIGHT_STATES_QUEUE_LENGTH 1

/* MQTT constants */
#define MQTT_SERVER "192.168.100.2"
#define MQTT_PORT 1882

/* WIFI credentials */
// const char* SSID = "Nakuja";
// const char* PASSWORD = "987654321";
const char* SSID = "Happyhome";
const char* PASSWORD = "JMongare@123";

/* ROCKET FLIGHT STATES */
#define PRE_FLIGHT          0
#define POWERED_FLIGHT      1
#define COASTING            2
#define APOGEE              3
#define BALLISTIC_DESCENT   4
#define PARACHUTE_DESCENT   5
#define POST_FLIGHT         6

/* LEDs for testing - remove on production */
#define PRE_FLIGHT_LED 4
int counter = 0;

//define the thresholds of the various displacements that need to be achieved in the state machine
#define GROUND_STATE_DISPLACEMENT 10
#define BELOW_APOGEE_LEVEL_DISPLACEMENT 10

// Pin to start ejection charge
#define EJECTION_PIN 4


extern float BASE_ALTITUDE;
extern float MAX_ALTITUDE;

#endif
#define TX 17
#define RX 16