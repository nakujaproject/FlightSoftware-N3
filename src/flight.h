#pragma once

#include "sensors/sensor.h"
#include "common/logging.h"

#define MAX_SENSORS 30

#if ENABLE_LOGGING
#define LOG_SENSOR(sensor) sensor->log()
#else
#define LOG_SENSOR(sensor)
#endif

class FlightSystem{
private:
    static FlightSystem* _instance;
    Sensor* sensors = nullptr, *_tail = nullptr;
    void readSensors();
public:
    FlightSystem();
    void addSensor(Sensor* sensor);
    void init();
    void run();
    static FlightSystem* getDefault();
};
