#pragma once

#include "common/tags.h"


class Comms {
private:
public:
    Comms() = default;
    virtual void send(ModelTag, const char* buffer) = 0;
};
