#include "flight.h"
#include "common/logging.h"
#include "sensors/MPU6050.h"

#define TAG "FLIGHT"

FlightSystem* FlightSystem::_instance = nullptr;

FlightSystem::FlightSystem() {
    _instance = this;
}

void FlightSystem::init() {
    LOGI("----------- STARTING FLIGHT SYSTEM -------------");

    addSensor(new MPU6050());
}

FlightSystem *FlightSystem::getDefault() {
    return _instance;
}

void FlightSystem::addSensor(Sensor* sensor) {
    if(sensor->init() != SENSOR_INITIALIZED){
        LOGE("Could not initialize sensor <%s>", sensor->getName().c_str());
        return;
    }
    if(_tail == nullptr){
        sensors = sensor;
        _tail = sensors;
        return;
    }
    _tail->next = sensor;
    _tail = sensor;
    _tail->next = nullptr;
}

void FlightSystem::readSensors() {
    Sensor* sensor = sensors;
    while(sensor){
        sensor->read();
        LOG_SENSOR(sensor);
        sensor = sensor->next;
    }
}

void FlightSystem::run() {
    readSensors();
    delay(500);
}
