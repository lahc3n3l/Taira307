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

        // Get the gyro bias vector
        BLA::Matrix<3, 1> getBias() const;

        // Get the quaternion vector
        BLA::Matrix<4, 1> getQuaternion() const ;

        // Get the attitude vector (roll, pitch, yaw)
        BLA::Matrix<3, 1> getAttitude() const ;

        // TODO: add propagate() and update() functions
        void predict(const BLA::Matrix<3, 1> &gyro, float dt);
        void propagate(const BLA::Matrix<3, 1>& gyro, float dt );
        void update(const BLA::Matrix<3, 1>& accel);
        void initializeFromAccel(const BLA::Matrix<3, 1>& accel );

    private:
        bool m_isInitialized = false; // Flag to check if the filter is initialized
        BLA::Matrix<4, 4> omegaMatrix(const BLA::Matrix<3, 1>& omega) const;
        void normalizeQuaternionInState();
        void normalizeQuaternion() ;
        BLA::Matrix<7, 7> computeJacobian(const BLA::Matrix<3, 1>& gyro, float dt) const;
        BLA::Matrix<3, 3> quaternionToRotationMatrix(const BLA::Matrix<4, 1>& q) const;

};
#endif
