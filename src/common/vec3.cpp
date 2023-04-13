#include "vec3.h"

vec3::vec3() : x(0), y(0), z(0) {}

vec3::vec3(float x, float y, float z) : x(x), y(y), z(z) {}

vec3::vec3(const vec3 &other) {
    x = other.x;
    y = other.y;
    z = other.z;
}

vec3 &vec3::operator=(const vec3 &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

vec3 vec3::operator+(const vec3 &other) const {
    return {x + other.x, y + other.y, z + other.z};
}

vec3 vec3::operator-(const vec3 &other) const {
    return {x - other.x, y - other.y, z - other.z};
}

vec3 vec3::operator*(float scalar) const {
    return {x * scalar, y * scalar, z * scalar};
}

bool vec3::operator==(const vec3 &other) const {
    return x == other.x && y == other.y && z == other.z;
}

bool vec3::operator!=(const vec3 &other) const {
    return !(operator==(other));
}
