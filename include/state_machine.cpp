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

// This checks that we have started ascent
// compares the current displacement to the set threshold of the ground state displacement
//if found to be above, we have achieved lift off
int powered_flight(float altitude)
{
    float displacement = altitude - BASE_ALTITUDE;
    if (displacement > BASE_ALTITUDE)
    {
        return POWERED_FLIGHT;
    }
    else
    {
        return PRE_FLIGHT;
    }
}

// This checks that we have reached apogee
// At apogee velocity is zero so we check for velocity less than or equal to zero
// we also check if the current altitude is less than the previous altitude 
//this would determine that the rocket has began descent
int apogee(float velocity, float currentAltitude, float previousAltitude)
{
    if ((previousAltitude - currentAltitude) > 5)
    {

        MAX_ALTITUDE = currentAltitude;
        return BALLISTIC_DESCENT;
    }
    else if (velocity <= 0)
    {
        MAX_ALTITUDE = currentAltitude;
        return BALLISTIC_DESCENT;
    }
    else
    {
        return COASTING;
    }
}

// Deploys parachute if we moved down below the set threshold for below apogee displacement
int parachute_deploy(float altitude)
{
    float displacement = MAX_ALTITUDE - altitude;
    if (displacement > BELOW_APOGEE_LEVEL_DISPLACEMENT)
    {
        // the ejection function is called which fires the ejection charge
        digitalWrite(EJECTION_PIN,HIGH);
        return PARACHUTE_DESCENT;
    }
    else
    {
        return BALLISTIC_DESCENT;
    }
}

// This checks that we have reached the ground
// detects landing of the rocket
// TODO: BASE_ALTITUDE might be different from the original base altitude
int post_flight(float altitude)
{
    float displacement = altitude - BASE_ALTITUDE;
    if (displacement < GROUND_STATE_DISPLACEMENT)
    {
        return POST_FLIGHT;
    }
    else
    {
        return BALLISTIC_DESCENT;
    }
}
