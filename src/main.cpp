#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>
#include "defs.h"

/* create gyroscope object */
Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

/* data variables */
/* altimeter data */
typedef struct {
    float pressure;
} AltimeterData;

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t altimeter_data_queue;

void _initialize_gyroscope(){
    /* attempt to initialize MPU6050 */
    if(!gyroscope.begin(0x68)){
        debugln("MPU6050 allocation failed!");

        // loop forever until found
        while(true){}
    }

    gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);
    gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);
    gyroscope.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(SETUP_DELAY);
}

void  _initialize_altimeter(){
    /* attempt to initialize BMP180 altimeter */
    if(!altimeter.begin(0x68)){
        debugln("Altimeter allocation failed!");

        // loop forever until found
        while(true){}
    }
}

void _readAltimeter(void* pvParameters){

    while(true){
        /* Read pressure.
         * This is the pressure from the sea level.
         * */
        AltimeterData alt_data;
        alt_data.pressure = altimeter.readSealevelPressure();

        /* Read altitude
         * This is the altitude from the sea level
         * */
//        altimeter.readAltitude(SEA_LEVEL_PRESSURE);

        /* send data to altimeter queue */
        if(xQueueSend(altimeter_data_queue, &alt_data, 0) != pdPASS){
            debugln("Queue full");
        }

        delay(TASK_DELAY);
    }
}

void _displayData(void* pvParameters){
    while(true){
        int buffer;

        if(xQueueReceive(altimeter_data_queue, &buffer, 0) == pdPASS){
            debugln(buffer);
        }else{
            /* no queue */
        }

        delay(TASK_DELAY);
    }
}


void setup()
{
    /* initialize serial */
    Serial.begin(115200);

    /* initialize sensors */
//    _initialize_gyroscope();
    _initialize_altimeter();
    // todo: initialize flash memory

    /* create altimeter_data_queue */
    altimeter_data_queue = xQueueCreate(ALTIMETER_QUEUE_LENGTH, sizeof(AltimeterData));

    /* Create tasks
     *
     * All tasks have a stack size of 1024 words - not bytes!
     * ESP32 is 32 bit, therefore 32bits x 1024 = 4096 bytes
     * So the stack size is 4096 bytes
     * */

    /* TASK 1: READ PRESSURE FROM THE ALTIMETER */
    xTaskCreatePinnedToCore(
            _readAltimeter,         /* function that executes this task*/
            "_readPressureFromAltimeter",/* Function name - for debugging */
            STACK_SIZE,                  /* Stack depth in words */
            NULL,                       /* parameter to be passed to the task */
            tskIDLE_PRIORITY + 1,        /* Task priority - in this case 1 */
            NULL,    /* task handle that can be passed to other tasks to reference the task */
            1   /* run on core 1*/
    );

    /* TASK 2: READ ACCELERATION FROM THE GYROSCOPE */

    /* TASK 3: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
    xTaskCreatePinnedToCore(
            _displayData,
            "_displayData",
            STACK_SIZE,
            NULL,
            tskIDLE_PRIORITY+1,
            NULL,
            1
            );
}

void loop(){
}