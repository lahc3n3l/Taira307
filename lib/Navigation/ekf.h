#ifndef EKF_H
#define EKF_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class EKF {

    public:
        // State vector: [q0, q1, q2, q3, bx, by, bz]
        BLA::Matrix<7, 1> X;

        // Covariance matrix P (7x7)
        BLA::Matrix<7, 7> P;

        // Process noise Q (7x7)
        BLA::Matrix<7, 7> Q;

        // Measurement noise R (3x3)
        BLA::Matrix<3, 3> R;

        // Constructor
        EKF() ;

        // Normalize the quaternion part of the state
        void normalizeQuaternion() ;

        // Get the gyro bias vector
        BLA::Matrix<3, 1> getBias() const;

        // Get the quaternion vector
        BLA::Matrix<4, 1> getQuaternion() const ;

        // TODO: add predict() and update() functions

    private:
        bool m_isInitialized = false; // Flag to check if the filter is initialized
        void initializeFromAccel(const BLA::Matrix<3, 1>& accel);
};
#endif
