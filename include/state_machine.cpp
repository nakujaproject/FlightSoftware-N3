/**
 * @file state_machine.cpp
 * @author Edwin Mwiti 
 * @brief 
 * @version 0.1
 * @date 2023-03-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <Arduino.h>
#include "state_machine.h"
#include "defs.h"

// variable for detected apogee height
float MAX_ALTITUDE = 0;
float PREVIOUS_ALTITUDE = 0;
float ALTITUDE_BUFFER[5];
int ALTITUDE_INDEX = 0;

//
bool pre_flight(float altitude){
    if(BASE_ALTITUDE-altitude<5) return true;
    return false;
}

// This checks that we have started ascent
// compares the current displacement to the set threshold of the ground state displacement
//if found to be above, we have achieved lift off
bool powered_flight(float altitude)
{
    if (BASE_ALTITUDE-altitude>5)  return true;
    return false;
}

// This checks that we have reached apogee
// At apogee velocity is zero so we check for velocity less than or equal to zero
// we also check if the current altitude is less than the previous altitude 
//this would determine that the rocket has began descent
bool apogee(float altitude){
}

// This checks that we have reached the ground
// detects landing of the rocket
// TODO: ALTITUDE_OFFSET might be different from the original base altitude
bool post_flight(float altitude)
{
    float displacement = altitude - BASE_ALTITUDE;
    if (displacement < BASE_ALTITUDE)
    {
        return POST_FLIGHT;
    }
    else
    {
        return BALLISTIC_DESCENT;
    }
}

bool ballistic_descent(float altitude){}
bool parachute_descent(float altitude){}

int checkState(float altitude){
    if(ALTITUDE_INDEX==5) ALTITUDE_INDEX=0;
    ALTITUDE_BUFFER[ALTITUDE_INDEX] = altitude;
    ALTITUDE_INDEX++;
    if(pre_flight(altitude))return POWERED_FLIGHT;
    if(powered_flight(altitude))return POWERED_FLIGHT;
    if(apogee(altitude)) return APOGEE;
    if(ballistic_descent(altitude)) return BALLISTIC_DESCENT;
    if(parachute_descent(altitude)) return PARACHUTE_DESCENT;
    if(post_flight(altitude)) return POST_FLIGHT;
}
