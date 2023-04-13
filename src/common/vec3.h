class vec3 {
public:
    union {
        float v[3];
        struct {
            float x, y, z;
        };
        struct {
            float roll, pitch, heading;
        };
    };

    vec3();
    vec3(float x, float y, float z);
    vec3(const vec3& other);
    vec3& operator=(const vec3& other);
    vec3 operator+(const vec3& other) const;
    vec3 operator-(const vec3& other) const;
    vec3 operator*(float scalar) const;
    bool operator==(const vec3& other) const;
    bool operator!=(const vec3& other) const;
};
