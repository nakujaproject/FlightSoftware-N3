#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>
#include "sensors.h"
#include "defs.h"

/* create Wi-Fi Client */
WiFiClient wifi_client;

/* create MQTT publish-subscribe client */
PubSubClient mqtt_client(wifi_client);

/* create gyroscope object */
Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

/* integration variables */
long long current_time = 0;
long long previous_time = 0;

/* velocity integration variables */
double y_velocity = 0;
double y_displacement = 0;

float new_y_displacement = 0.0;
float old_y_displacement = 0.0;
double old_y_velocity = 0.0;
double new_y_velocity = 0.0;
double total_y_displacement = 0.0;

/* functions to initialize sensors */
void initialize_gyroscope(){
    /* attempt to initialize MPU6050 */
    if(!gyroscope.begin(0x68)){
        debugln("[-]Gyroscope allocation failed!");
        // loop forever until found
        while(true){}
    }

    debugln("[+]Gyroscope Initialized");
    gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);
    gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);
    gyroscope.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(SETUP_DELAY);
}

void initialize_altimeter(){
    if (!altimeter.begin()) {
        debugln("[-]Could not find a valid altimeter sensor");
        while (1) {}

    }

    debugln("[+]Altimeter initialized");
}

/* data variables */
/* gyroscope data */

struct Acceleration_Data{
    float ax;
    float ay; 
    float az;
};

struct Altimeter_Data{
    int32_t pressure;
    float altitude;
    double velocity;
    double AGL; /* altitude above ground level */
};

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t gyroscope_data_queue;
QueueHandle_t altimeter_data_queue;
QueueHandle_t filtered_data_queue;

void connectToWifi(){
    /* Connect to a Wi-Fi network */
    debugln("[..]Scanning for network...");

    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        debugln("[..]Scanning for network...");
    }

    debugln("[+]Network found");debug("[+]My IP address: "); debugln();
    debugln(WiFi.localIP());
}

void initializeMQTTParameters(){
    /* this functions creates an MQTT client for transmitting telemetry data */
    mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
}

void readAltimeter(void* pvParameters){

    while(true){
        /* Read pressure.
         * This is the pressure from the sea level.
         * */
        struct Altimeter_Data altimeter_data;

        /* Read pressure
         * This is the pressure from the sea level
         * */
        altimeter_data.pressure = altimeter.readSealevelPressure();

        /* Read altitude
         * This is the altitude from the sea level
         * */
        altimeter_data.altitude = altimeter.readAltitude(SEA_LEVEL_PRESSURE);

        /*------------- APOGEE DETECTION ALGORITHM -------------------------------------*/

        /* approximate velocity from acceleration by integration for apogee detection */
        current_time = millis();

        /* differentiate displacement to get velocity */
        new_y_displacement = altimeter_data.altitude - ALTITUDE_OFFSET;
        y_velocity = (new_y_displacement - old_y_displacement) / (current_time - previous_time);

        /* update integration variables */
        previous_time = current_time;
        old_y_displacement = new_y_displacement;

        /* ------------------------ END OF APOGEE DETECTION ALGORITHM ------------------------ */

        /* subtract current altitude to get the maximum height reached */
        float rocket_height = altimeter_data.altitude - ALTITUDE_OFFSET;

        /* update altimeter data */
        altimeter_data.velocity = y_velocity;
        altimeter_data.AGL = rocket_height;

        /* send data to altimeter queue */
        if(xQueueSend(altimeter_data_queue, &altimeter_data, portMAX_DELAY) != pdPASS){
            debugln("[-]Altimeter queue full");
        }

        delay(TASK_DELAY);
    }
}

void readGyroscope(void* pvParameters){
    
    while(true){
        sensors_event_t a, g, temp;
        gyroscope.getEvent(&a, &g, &temp);
        
        struct Acceleration_Data gyro_data;
        /* 
        * Read accelerations on all axes
         * */
        gyro_data.ax = a.acceleration.x;
        gyro_data.ay = a.acceleration.y;
        gyro_data.az = a.acceleration.z;

        /* send data to gyroscope queue */
        if(xQueueSend(gyroscope_data_queue, &gyro_data, portMAX_DELAY) != pdPASS){
            debugln("[-]Gyro queue full");
        }

        delay(TASK_DELAY);
    }
}

void displayData(void* pvParameters){
   while(true){
       struct Acceleration_Data gyroscope_buffer;
       struct Altimeter_Data altimeter_buffer;

       if(xQueueReceive(gyroscope_data_queue, &gyroscope_buffer, portMAX_DELAY) == pdPASS){
           debugln("------------------------------");
            debug("x: "); debug(gyroscope_buffer.ax); debugln();
            debug("y: "); debug(gyroscope_buffer.ay); debugln();
            debug("z: "); debug(gyroscope_buffer.az); debugln();

       }else{
           /* no queue */
       }

        if(xQueueReceive(altimeter_data_queue, &altimeter_buffer, portMAX_DELAY) == pdPASS){
            debug("Pressure: "); debug(altimeter_buffer.pressure); debugln();
            debug("Altitude: "); debug(altimeter_buffer.altitude); debugln();
            debug("Velocity: "); debug(altimeter_buffer.velocity); debugln();
            debug("AGL: "); debug(altimeter_buffer.AGL); debugln();
           
        }else{
            /* no queue */
        }

       delay(10);
   }
}

void publishMQTTMessage(struct Altimeter_Data altimeter_data, struct Acceleration_Data acceleration_data){
    /* This function creates the message to be sent to the ground station over MQTT*/
    char telemetry_data[300];

    /* build telemetry string message
     * The data to be sent is:
     *
     * y-axis acceleration
     * Velocity
     * Altitude above ground level (AGL)
     * Pressure
     *
     * */
    sprintf(telemetry_data,
            "%.3f, %.3f, %.3f, %.3f\n",
            acceleration_data.ax,
            acceleration_data.ay,
            acceleration_data.az,
            altimeter_data.velocity,
            altimeter_data.AGL,
            altimeter_data.pressure
            );

    /* publish the data to N3/TelemetryData MQTT channel */
//    mqtt_client.publish("n3/telemetry-data", "y-acceleration, velocity, altitude, pressure");
    mqtt_client.publish("n3/telemetry-data", telemetry_data);
    // mqtt_client
}

void transmitTelemetry(void* pvParameters){
    /* This function sends data to the ground station */
    while(true){
        /*  create two pointers to the data structures to be transmitted */
        struct Acceleration_Data gyroscope_data_receive;
        struct Altimeter_Data altimeter_data_receive;

        /* receive data into respective queues */
        if( (xQueueReceive(gyroscope_data_queue, &gyroscope_data_receive, portMAX_DELAY) == pdPASS) || (xQueueReceive(altimeter_data_queue, &altimeter_data_receive, portMAX_DELAY) == pdPASS) ){
            /* this means all the data has been received successfully into any one of the queues and is ready for transmission
             *
             * todo: change this OR operator. the danger is that if one queue is not able to receive, we will not receive any data
             *
             *
             * */

            /* publish the data to ground */

//            char telemetry_data[300];

            /* build telemetry string message
             * The data to be sent is:
             *
             * y-axis acceleration
             * Velocity
             * Altitude above ground level (AGL)
             * Pressure
             *
             * */
//            sprintf(telemetry_data,
//                    "%.3f, %.3f, %.3f, %.3f\n",
//                    gyroscope_data_receive.ay,
//                    altimeter_data_receive.velocity,
//                    altimeter_data_receive.AGL,
//                    altimeter_data_receive.pressure
//            );

            debugln("[+]Sending data");

            /* publish the data to N3/TelemetryData MQTT channel */

            mqtt_client.publish("n3/telemetry-data", "Hello from flight!");
//            mqtt_client.publish("n3/telemetry-data", "y-acceleration, velocity, altitude, pressure");
//            mqtt_client.publish("N3/TelemetryData", telemetry_data);

        }else{
            debugln("[-]Failed to receive data for sending");
        }
    }
}

void testTelemetry(void * pvParameter){
    while(1){
        mqtt_client.publish("n3/telemetry-data", "Hello from flight!");
    }
}

void reconnect()
{
    while (!mqtt_client.connected())
    {
        // Serial.begin(115200);
        debug("Attempting MQTT connection...");
        String clientId = "FCClient-";
        clientId += String(random(0xffff), HEX);
        if (mqtt_client.connect(clientId.c_str()))
        {
            debugln("Connected");
            mqtt_client.publish("n3/telemetry-data","Connected");
        }
        else
        {
            debug("failed,rc=");
            debug(mqtt_client.state());
            debugln(" reconnecting");
            delay(500);
        }
    }
}

void callback(char *topic, byte* message, unsigned int length) {
//  client.subscribe(topic);
//  client.publish(inTopic, "We are on ma nigga");

}


void setup(){
    /* initialize serial */
    Serial.begin(115200);

    /* connect to WiFi*/
    connectToWifi();

    /* initialize sensors */
    initialize_gyroscope();
    initialize_altimeter();
    // todo: initialize flash memory

    mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt_client.setCallback(callback);


    debugln("Creating queues...");

    /* create gyroscope data queue */
    gyroscope_data_queue = xQueueCreate(GYROSCOPE_QUEUE_LENGTH, sizeof(struct Acceleration_Data));

    /* create altimeter_data_queue */   
    altimeter_data_queue = xQueueCreate(ALTIMETER_QUEUE_LENGTH, sizeof(struct Altimeter_Data));

    /* check if the queues were created successfully */
    if(gyroscope_data_queue == NULL){
        debugln("[-]Gyroscope data queue creation failed!");
    } else{
        debugln("[+]Gyroscope data queue creation success");
    }
    
    if(altimeter_data_queue == NULL){
        debugln("[-]Altimeter data queue creation failed!");
    } else{
        debugln("[+]Altimeter data queue creation success");
    }

    if(filtered_data_queue == NULL){
        debugln("[-]Filtered data queue creation failed!");
    } else{
        debugln("[+]Filtered data queue creation success");
    }
    
    /* Create tasks
     * All tasks have a stack size of 1024 words - not bytes!
     * ESP32 is 32 bit, therefore 32bits x 1024 = 4096 bytes
     * So the stack size is 4096 bytes
     * */
    debugln("Creating tasks...");

    /* TASK 1: READ ALTIMETER DATA */
   if(xTaskCreate(
           readAltimeter,               /* function that executes this task*/
           "readAltimeter",             /* Function name - for debugging */
           STACK_SIZE,                  /* Stack depth in words */
           NULL,                        /* parameter to be passed to the task */
           2,        /* Task priority - in this case 1 */
           NULL                         /* task handle that can be passed to other tasks to reference the task */
   ) != pdPASS){
    // if task creation is not successful
    debugln("[-]Read-Altimeter task creation failed!");

   }else{
    debugln("[+]Read-Altimeter task creation success");
   }

    /* TASK 2: READ GYROSCOPE DATA */
   if(xTaskCreate(
           readGyroscope,         
           "readGyroscope",
           STACK_SIZE,                  
           NULL,                       
           2,
           NULL    
   ) != pdPASS){
    debugln("[-]Read-Gyroscope task creation failed!");
   } else{
    debugln("[+]Read-Gyroscope task creation success!");
   }

    /* TASK 3: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
//    if(xTaskCreate(
//            displayData,
//            "displayData",
//            STACK_SIZE,
//            NULL,
//            2,
//            NULL
//            ) != pdPASS){
//     debugln("[-]Display data task creation failed!");
//     }else{
//     debugln("[+]Display data task creation success!");
//     }

    /* TASK 4: TRANSMIT TELEMETRY DATA */
    if(xTaskCreate(
            testTelemetry,
            "test",
            STACK_SIZE,
            NULL,
            1,
            NULL
    ) != pdPASS){
        debugln("[-]Display data task creation failed!");
    }else{
        debugln("[+]Display data task creation success!");
    }

}

void loop(){
    mqtt_client.publish("n3/telemetry-data", "Hello from flight");
     if (!mqtt_client.connected())  // Reconnect if connection is lost
    {
      reconnect();
    }
    mqtt_client.loop();
    delay(1);
}
