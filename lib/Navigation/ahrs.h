#ifndef AHRS_H
#define AHRS_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class AHRS {
public:
    // Quaternion structure
    struct Quaternion {
        float w, x, y, z;

        Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
        Quaternion(float _w, float _x, float _y, float _z)
            : w(_w), x(_x), y(_y), z(_z) {}
    };

    // Default constructor
    AHRS() : m_roll(0.0f), m_pitch(0.0f), m_yaw(0.0f), isInitialized(false),
             orientation(1.0f, 0.0f, 0.0f, 0.0f) {}

    // Initialize with accelerometer data
    void init(Matrix<3, 1> accel) {
        m_roll = atan2(accel(1), accel(2));
        m_pitch = atan2(-accel(0), sqrt(accel(1) * accel(1) + accel(2) * accel(2)));
        m_yaw = 0.0f;

        // Update quaternion based on estimated Euler angles
        updateQuaternionFromEuler(m_roll, m_pitch, m_yaw);

        isInitialized = true;
    }

    // Get the current orientation quaternion
    Quaternion getQuaternion() const {
        return orientation;
    }

private:
    // Member variables
    bool isInitialized;

    float m_roll;
    float m_pitch;
    float m_yaw;

    Quaternion orientation;

    // Helper function to update quaternion from Euler angles
    void updateQuaternionFromEuler(float roll, float pitch, float yaw) {
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);

        orientation.w = cr * cp * cy + sr * sp * sy;
        orientation.x = sr * cp * cy - cr * sp * sy;
        orientation.y = cr * sp * cy + sr * cp * sy;
        orientation.z = cr * cp * sy - sr * sp * cy;
    }
};

#endif
