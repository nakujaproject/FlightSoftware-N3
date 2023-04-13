#include "sensor.h"

SensorState Sensor::state() {
    return _state;
}

void Sensor::read() {
    read_time = millis();
    _read();
}

String Sensor::getName() {
    return name;
}
