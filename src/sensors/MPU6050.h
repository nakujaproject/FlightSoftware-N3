#pragma once

#include <cstdint>
#include <Adafruit_MPU6050.h>

#include "common/vec3.h"
#include "sensor.h"
#include "protogen/sensors.pb.h"

#define MPU6050_I2CADDR MPU6050_I2CADDR_DEFAULT


class MPU6050: public Sensor{
private:
    uint8_t i2cAddr;
    Adafruit_MPU6050 mpu;
    flight_GyroReading reading = {};
public:
    explicit MPU6050(uint8_t i2c_addr = MPU6050_I2CADDR);

    SensorState test() override;

    void _read() override;

    void log() override;

    SensorState init() override;

    SensorReading get() override;
};
