#include "flight.h"
#include "esp_log.h"

#define TAG "MAIN"

void setup() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    auto flight = new FlightSystem();
    flight->init();
}

void loop() {
    FlightSystem::getDefault()->run();
}