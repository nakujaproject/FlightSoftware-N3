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

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t altimeter_data_queue;

void _initialize_gyroscope(){
    /* attempt to initialize MPU6050 */
    if(!gyroscope.begin(0x68)){
        debugln("MPU6050 allocation failed!");

        // loop forever until found
        for(;;);
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
        altimeter.readSealevelPressure();

        /* Read altitude
         * This is the altitude from the sea level
         * */
        altimeter.readAltitude(SEA_LEVEL_PRESSURE);
    }
}

void setup()
{
    /* initialize serial */
    Serial.begin(115200);

    /* initialize sensors */
    _initialize_gyroscope();
    _initialize_altimeter();
    // todo: initialize flash memory

    
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
            (void *) altimeter_arr[], /* parameter to be passed to the task */
            tskIDLE_PRIORITY + 1,        /* Task priority - in this case 1 */
            &readAltimeterTaskHandle    /* task handle that can be passed to other tasks to reference the task */
    );

    /* TASK 2: READ ACCELERATION FROM THE GYROSCOPE */


}

void loop()
{

}