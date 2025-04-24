#include <BasicLinearAlgebra.h>
#include <math.h>


class KalmanFilter {
public:
    KalmanFilter();

    void predict(const BLA::Matrix<3>& u, float dt);
    void update(const BLA::Matrix<3>& accel);
    void updateWithGnss(float course, float headingAccEst); ;

    void getEulerAnglesDeg(float &roll, float &pitch, float &yaw) const;
    float getRollRad() const { return x(0); }
    float getPitchRad() const { return x(1); }
    float getYawRad() const { return x(2); }

    float getRollDeg() const { return x(0) * (float)RAD_TO_DEG; }
    float getPitchDeg() const { return x(1) * (float)RAD_TO_DEG; }
    float getYawDeg() const { return x(2) * (float)RAD_TO_DEG; }
    void initializeYaw(float yaw) { x(2) = yaw; } // Initialize yaw with GNSS heading

private:
    BLA::Matrix<3,3> A, B, Q, P;
    BLA::Matrix<2,3> H;
    BLA::Matrix<2,2> R;
    BLA::Matrix<3> x;
};
