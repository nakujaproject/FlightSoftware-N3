#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "defs.h"

class State_machine{

    public:
        int32_t pre_flight(float altitude);
        int32_t powered_flight(float altitude);
        int32_t coasting(float altitude, float velocity);
        int32_t apogee(float velocity, float currentAltitude, float previousAltitude);
        int32_t ballistic_descent(float altitude);
        int32_t parachute_deploy(float altitude);
        int32_t post_flight(float altitude, float velocity);
};

/* class members declaration */

int32_t State_machine::pre_flight(float altitude){
    return PRE_FLIGHT;
}

int32_t State_machine::powered_flight(float altitude){
    return POWERED_FLIGHT;
}

int32_t State_machine::coasting(float altitude, float velocity){
    return COASTING;
}

int32_t State_machine::apogee(float velocity, float currentAltitude, float previousAltitude){
    return APOGEE;
}

int32_t State_machine::ballistic_descent(float altitude){
    return BALLISTIC_DESCENT;
}

int32_t State_machine::parachute_deploy(float altitude){
    return PARACHUTE_DESCENT;
}

int32_t State_machine::post_flight(float altitude, float velocity){
    return POST_FLIGHT;
}



#endif
