#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>
#include "defs.h"
#include "sensors.h"

/* create gyroscope object */
Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

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
        for(;;);
    }
}



void setup()
{
    /* initialize serial */
    Serial.begin(115200);

    /* initialize sensors */
    _initialize_gyroscope()
    _initialize_altimeter();
    
    /* Create tasks */
    /* TASK 1: Read the pressure from the altimeter */
    xTaskCreatePinnedToCore(

    );

}

void loop()
{

}