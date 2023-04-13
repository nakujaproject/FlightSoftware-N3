#include "common/logging.h"
#include "common/tags.h"
#include "MPU6050.h"

#define TAG "MPU6050"


MPU6050::MPU6050(uint8_t i2c_addr) : i2cAddr(i2c_addr){
    name = "MPU6050";
    tag = SENSOR_GYRO;
}

SensorState MPU6050::init() {
    if(mpu.begin(i2cAddr)){
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        LOGI("Initialization successful");
        return SENSOR_INITIALIZED;
    }
    LOGE("Initialization failed");
    return SENSOR_ERROR;
}

SensorState MPU6050::test() {
    return SENSOR_OK;
}

void MPU6050::_read() {
    sensors_event_t a, g, temp;
    if(mpu.getEvent(&a, &g, &temp)) {
        reading.a = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
        reading.g = {g.gyro.x, g.gyro.y, g.gyro.z};
    }
}

SensorReading MPU6050::get() {
    return &reading;
}

void MPU6050::log() {
    LOGD(
        "acc: (%f, %f, %f), gyro: (%, %f, %f)",
        reading.a.x, reading.a.y, reading.a.z,
        reading.g.x, reading.g.y, reading.g.z
    );
}
