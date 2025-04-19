#include "EKF.h"
#include <cmath>

EKF::EKF() {
    // Initialize state to identity quaternion and zero biases
    X = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Initialize matrices to zero
    P.Fill(0.0f);
    
    Q.Fill(0.0f);
    Q(0, 0) = 0.001f; // q0
    Q(1, 1) = 0.001f; // q1
    Q(2, 2) = 0.001f; // q2
    Q(3, 3) = 0.001f; // q3
    Q(4, 4) = 0.0f;   // bx
    Q(5, 5) = 0.0f;   // by
    Q(6, 6) = 0.0f;   // bz

    R.Fill(0.0f);
    R(0, 0) = 0.03f; // accel x
    R(1, 1) = 0.03f; // accel y
    R(2, 2) = 0.03f; // accel z
}

void EKF::normalizeQuaternion() {
    float norm = sqrt(state(0)*state(0) + state(1)*state(1) +
                      state(2)*state(2) + state(3)*state(3));
    if (norm > 0.0f) {
        state(0) /= norm;
        state(1) /= norm;
        state(2) /= norm;
        state(3) /= norm;
    }
}

BLA::Matrix<3, 1> EKF::getBias() const {
    return {state(4), state(5), state(6)};
}

BLA::Matrix<4, 1> EKF::getQuaternion() const {
    return {state(0), state(1), state(2), state(3)};
}

void EKF::initializeFromAccel(const BLA::Matrix<3, 1>& accel) {
    // Normalize the accelerometer vector
    float norm = sqrt(accel(0)*accel(0) + accel(1)*accel(1) + accel(2)*accel(2));
    if (norm > 0.0f) {
        BLA::Matrix<3, 1> a = accel / norm;
        state(0) = sqrt(0.5f * (1.0f + a(2))); // q0
        state(1) = -a(1) * sqrt(0.5f / (1.0f + a(2))); // q1
        state(2) = a(0) * sqrt(0.5f / (1.0f + a(2))); // q2
        state(3) = 0.0f; // q3
    }
    else {
        // Handle the case where the accelerometer vector is zero
        state(0) = 1.0f; // q0
        state(1) = 0.0f; // q1
        state(2) = 0.0f; // q2
        state(3) = 0.0f; // q3
    }
}
