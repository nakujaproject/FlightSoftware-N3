#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

class State_machine{

    public:
        int pre_flight();
        int powered_flight();
        int coasting();
        int apogee();
        int ballistic_descent();
        int parachute_deploy();
        int post_flight();
};

/* class members declaration */

int32_t State_machine::pre_flight(){
    return PRE_FLIGHT;
}

int32_t State_machine::powered_flight(){
    return POWERED_FLIGHT;
}

int32_t State_machine::coasting(){
    return COASTING;
}

int32_t State_machine::apogee(){
    return APOGEE;
}

int32_t State_machine::ballistic_descent(){
    return BALLISTIC_DESCENT;
}

int32_t State_machine::parachute_deploy(){
    return PARACHUTE_DESCENT;
}

int32_t State_machine::post_flight(){
    return POST_FLIGHT;
}



#endif
