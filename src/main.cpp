#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>
#include "sensors.h"
#include "defs.h"

/* create gyroscope object */
Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

/* acceleration integration variables */
long long current_time = 0;
long long previous_time = 0;
long velocity = 0;
float old_y_acceleration = 0.0;
float new_y_acceleration = 0.0;


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
    long velocity;
};

struct Altimeter_Data{
    int32_t pressure;
    float altitude;
};

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t gyroscope_data_queue;
QueueHandle_t altimeter_data_queue;
QueueHandle_t filtered_data_queue;

void readAltimeter(void* pvParameters){

    while(true){
        /* Read pressure.
         * This is the pressure from the sea level.
         * */
        struct Altimeter_Data alt_data;

        /* Read pressure
         * This is the pressure from the sea level
         * */
        alt_data.pressure = altimeter.readSealevelPressure();

        /* Read altitude
         * This is the altitude from the sea level
         * */
        alt_data.altitude = altimeter.readAltitude(SEA_LEVEL_PRESSURE);

        /* send data to altimeter queue */
        if(xQueueSend(altimeter_data_queue, &alt_data, portMAX_DELAY) != pdPASS){
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

        /* approximate velocity from acceleration by integration for apogee detection */
        current_time = millis();
        new_y_acceleration = a.acceleration.y;
        velocity = (old_y_acceleration - new_y_acceleration)/2 * (current_time - previous_time);
        old_y_acceleration = new_y_acceleration;

        previous_time = current_time;
        old_y_acceleration = new_y_acceleration;

        /* assign velocity value to gyroscope data */
        gyro_data.velocity = velocity;

        /* send data to altimeter queue */
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
            debug("velocity: "); debug(gyroscope_buffer.velocity); debugln();

       }else{
           /* no queue */
       }

        if(xQueueReceive(altimeter_data_queue, &altimeter_buffer, portMAX_DELAY) == pdPASS){
            debug("pressure: "); debug(altimeter_buffer.pressure); debugln();
            debug("altitude: "); debug(altimeter_buffer.altitude); debugln();
           
        }else{
            /* no queue */
        }

       delay(10);
   }
}



void setup()
{
    /* initialize serial */
    Serial.begin(115200);

    /* initialize sensors */
    initialize_gyroscope();
    initialize_altimeter();
    // todo: initialize flash memory

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
           tskIDLE_PRIORITY + 1,        /* Task priority - in this case 1 */
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
           1,        
           NULL    
   ) != pdPASS){
    debugln("[-]Read-Gyroscope task creation failed!");
   } else{
    debugln("[+]Read-Gyroscope task creation success!");
   }

    /* TASK 3: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
   if(xTaskCreate(
           displayData,
           "displayData",
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
    delay(1);
}