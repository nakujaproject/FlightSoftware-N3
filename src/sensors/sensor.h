#pragma once

#include <Arduino.h>
#include "common/tags.h"


typedef void* SensorReading;

typedef enum SensorState {
    SENSOR_NOT_INITIALIZED = 0,
    SENSOR_INITIALIZED = 1,
    SENSOR_OK = 2,
    SENSOR_ERROR = 3,
} SensorState;


class Sensor{
private:
    unsigned long read_time = 0;
protected:
    String name = "";
    uint8_t tag = TAG_UNSET;
    SensorState _state = SENSOR_NOT_INITIALIZED;
    SensorReading _value = nullptr;
public:
    Sensor* next = nullptr;
    Sensor() = default;
    virtual SensorState init() = 0;
    virtual SensorState test() = 0;
    virtual SensorState state();

    virtual void read() final;
    virtual String getName() final;
    virtual void _read() = 0;
    virtual void log() = 0;
    virtual SensorReading get() = 0;
};