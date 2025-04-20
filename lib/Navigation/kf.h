#include <BasicLinearAlgebra.h>
#include <math.h>


class KalmanFilter {
public:
    KalmanFilter();

    void predict(const BLA::Matrix<3>& u, float dt);
    void update(const BLA::Matrix<3>& accel);
    void getEulerAnglesDeg(float &roll, float &pitch, float &yaw) const;

private:
    BLA::Matrix<3,3> A, B, Q, P;
    BLA::Matrix<2,3> H;
    BLA::Matrix<2,2> R;
    BLA::Matrix<3> x;
};
