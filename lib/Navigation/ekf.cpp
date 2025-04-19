#include "EKF.h"
#include <cmath>

EKF::EKF() {
    // Initialize X to identity quaternion and zero biases
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
    float norm = sqrt(X(0)*X(0) + X(1)*X(1) +
                      X(2)*X(2) + X(3)*X(3));
    if (norm > 0.0f) {
        X(0) /= norm;
        X(1) /= norm;
        X(2) /= norm;
        X(3) /= norm;
    }
}

BLA::Matrix<3, 1> EKF::getBias() const {
    return {X(4), X(5), X(6)};
}

BLA::Matrix<4, 1> EKF::getQuaternion() const {
    return {X(0), X(1), X(2), X(3)};
}

BLA::Matrix<3, 1> EKF::getAttitude() const {
    BLA::Matrix<3, 1> attitude;
    attitude(0) = atan2(2.0f * (X(0) * X(1) + X(2) * X(3)), 1.0f - 2.0f * (X(1) * X(1) + X(2) * X(2))); // Roll
    attitude(1) = asin(2.0f * (X(0) * X(2) - X(3) * X(1))); // Pitch
    attitude(2) = atan2(2.0f * (X(0) * X(3) + X(1) * X(2)), 1.0f - 2.0f * (X(2) * X(2) + X(3) * X(3))); // Yaw
    return attitude;
}

void EKF::predict(const BLA::Matrix<3, 1> &gyro, float dt)
{
 
        // Extract bias from state
        BLA::Matrix<3, 1> bias = {X(4), X(5), X(6)};
        BLA::Matrix<3, 1> omega = gyro - bias;
    
        // Quaternion part of state
        BLA::Matrix<4, 1> q = {X(0), X(1), X(2), X(3)};
        BLA::Matrix<4, 4> omegaMat = omegaMatrix(omega);
    
        // Quaternion derivative
        BLA::Matrix<4, 1> qDot = 0.5f * omegaMat * q;
    
        // Integrate quaternion
        BLA::Matrix<4, 1> qNew = q + qDot * dt;
    
        // Update quaternion in state
        X(0) = qNew(0);
        X(1) = qNew(1);
        X(2) = qNew(2);
        X(3) = qNew(3);

    
        // NOTE: Covariance update step is not included here but can be added later.
        // The covariance matrix P can be updated using the process noise Q and the Jacobian of the state transition function.

        propagate(gyro, dt); // Propagate the covariance matrix

}

void EKF::initializeFromAccel(const BLA::Matrix<3, 1> &accel)
{
    // Normalize the accelerometer vector
    float norm = sqrt(accel(0)*accel(0) + accel(1)*accel(1) + accel(2)*accel(2));
    if (norm > 0.0f) {
        BLA::Matrix<3, 1> a = accel / norm;
        X(0) = sqrt(0.5f * (1.0f + a(2))); // q0
        X(1) = -a(1) * sqrt(0.5f / (1.0f + a(2))); // q1
        X(2) = a(0) * sqrt(0.5f / (1.0f + a(2))); // q2
        X(3) = 0.0f; // q3
    }
    else {
        // Handle the case where the accelerometer vector is zero
        X(0) = 1.0f; // q0
        X(1) = 0.0f; // q1
        X(2) = 0.0f; // q2
        X(3) = 0.0f; // q3
    }
}


BLA::Matrix<4, 4> EKF::omegaMatrix(const BLA::Matrix<3, 1>& omega) const {
    float wx = omega(0);
    float wy = omega(1);
    float wz = omega(2);

    return  
        { 0,   -wx, -wy, -wz,
         wx,   0,   wz, -wy,
         wy,  -wz,  0,   wx,
         wz,   wy, -wx,  0  };
    };

void EKF::normalizeQuaternionInState() {
        float norm = sqrt(X(0)*X(0) + X(1)*X(1) + X(2)*X(2) + X(3)*X(3));
        if (norm > 0.0f) {
            X(0) /= norm;
            X(1) /= norm;
            X(2) /= norm;
            X(3) /= norm;
        }
    }

    BLA::Matrix<7, 7> EKF::computeJacobian(const BLA::Matrix<3, 1>& gyro, float dt) const {
        float q0 = X(0);
        float q1 = X(1);
        float q2 = X(2);
        float q3 = X(3);
        float bx = X(4);
        float by = X(5);
        float bz = X(6);
    
        float p = gyro(0) - bx;
        float q = gyro(1) - by;
        float r = gyro(2) - bz;
        
        // initialize the Jacobian matrix as an identity matrix
        
        BLA::Matrix<7, 7> f = BLA::Matrix<7,7> {1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,
                                                0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,1.0f};
                                            
    
        // Quaternion dynamics w.r.t quaternion
        f(0, 1) = -p * dt;
        f(0, 2) = -q * dt;
        f(0, 3) = -r * dt;
    
        f(1, 0) =  p * dt;
        f(1, 2) =  r * dt;
        f(1, 3) = -q * dt;
    
        f(2, 0) =  q * dt;
        f(2, 1) = -r * dt;
        f(2, 3) =  p * dt;
    
        f(3, 0) =  r * dt;
        f(3, 1) =  q * dt;
        f(3, 2) = -p * dt;
    
        // Quaternion dynamics w.r.t bias
        f(0, 4) =  0.5f * q1 * dt;
        f(0, 5) =  0.5f * q2 * dt;
        f(0, 6) =  0.5f * q3 * dt;
    
        f(1, 4) = -0.5f * q0 * dt;
        f(1, 5) = -0.5f * q3 * dt;
        f(1, 6) =  0.5f * q2 * dt;
    
        f(2, 4) =  0.5f * q3 * dt;
        f(2, 5) = -0.5f * q0 * dt;
        f(2, 6) = -0.5f * q1 * dt;
    
        f(3, 4) = -0.5f * q2 * dt;
        f(3, 5) =  0.5f * q1 * dt;
        f(3, 6) = -0.5f * q0 * dt;
    
        return f;
    }

    void EKF::propagate(const BLA::Matrix<3, 1>& gyro, float dt ) {
        // Step 1: Compute the Jacobian matrix F
        BLA::Matrix<7, 7> F = computeJacobian(gyro, dt);
        // Propagate the covariance matrix
        BLA::Matrix<7, 7> P = F * P * (~F); // Propagate the covariance matrix
    }

    BLA::Matrix<3, 3> EKF::quaternionToRotationMatrix(const BLA::Matrix<4, 1> &q) const
    {
    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    BLA::Matrix<3, 3> R;
    R(0, 0) = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    R(0, 1) = 2.0f * (q1 * q2 - q0 * q3);
    R(0, 2) = 2.0f * (q1 * q3 + q0 * q2);

    R(1, 0) = 2.0f * (q1 * q2 + q0 * q3);
    R(1, 1) = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    R(1, 2) = 2.0f * (q2 * q3 - q0 * q1);

    R(2, 0) = 2.0f * (q1 * q3 - q0 * q2);
    R(2, 1) = 2.0f * (q2 * q3 + q0 * q1);
    R(2, 2) = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    return R;
    }

    void EKF::update(const BLA::Matrix<3, 1>& accel) {
        const float GRAVITY = 9.81f;
    
        // Step 1: Compute expected gravity vector in sensor frame
        BLA::Matrix<3, 1> gravity = { 0.0f, 0.0f, -GRAVITY };
        BLA::Matrix<4, 1> q = { X(0), X(1), X(2), X(3) };
    
        BLA::Matrix<3, 3> R = quaternionToRotationMatrix(q);
        BLA::Matrix<3, 1> accelExpected = ~R * gravity;
    
        // Step 2: Compute innovation: z - h(x)
        BLA::Matrix<3, 1> innovation = accel - accelExpected;
    
        // Step 3: Compute measurement Jacobian H (3x7)
        BLA::Matrix<3, 7> H;
        float q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
    
        H.Fill(0.0f);
        H(0, 0) = -2 * GRAVITY * q2;
        H(0, 1) =  2 * GRAVITY * q3;
        H(0, 2) = -2 * GRAVITY * q0;
        H(0, 3) =  2 * GRAVITY * q1;
    
        H(1, 0) =  2 * GRAVITY * q1;
        H(1, 1) =  2 * GRAVITY * q0;
        H(1, 2) =  2 * GRAVITY * q3;
        H(1, 3) =  2 * GRAVITY * q2;
    
        H(2, 0) =  2 * GRAVITY * q0;
        H(2, 1) = -2 * GRAVITY * q1;
        H(2, 2) = -2 * GRAVITY * q2;
        H(2, 3) = -2 * GRAVITY * q3;
    
        // Step 4: Compute innovation covariance: S = HPHᵀ + R
        BLA::Matrix<3, 3> S = H * P * ~H + R;
    
        // Step 5: Compute Kalman gain: K = PHᵀS⁻¹
        BLA::Matrix<3, 3> S_inv = Invert(S);
        BLA::Matrix<7, 3> K = P * ~H * S_inv;
    
        // Step 6: Update state estimate: X = X + K * innovation
        X += K * innovation;
    
        // Step 7: Update covariance: P = (I - K * H) * P
        BLA::Matrix<7, 7> I = BLA::Matrix<7, 7> {1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f, 0.0f,1.0f,0.0f,0.0f,
                                                0.0f,0.0f,0.0f, 0.0f,0.0f,1.0f,0.0f,
                                                0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f,1.0f};
        P = (I - K * H) * P;
    
        // Step 8: Normalize quaternion
        float norm = sqrt(X(0)*X(0) + X(1)*X(1) + X(2)*X(2) + X(3)*X(3));
        for (int i = 0; i < 4; ++i) {
            X(i) /= norm;
        }
    }
// End of EKF.cpp    


