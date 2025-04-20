#include "kf.h"

KalmanFilter::KalmanFilter() {
    // A is identity as float
    A = {1.0f,0.0f,0.0f,
         0.0f,1.0f,0.0f,
         0.0f,0.0f,1.0f};

    // H maps roll and pitch from state x :float
    // to measurement y :float
    H = {1.0f,0.0f,0.0f,
         0.0f,1.0f,0.0f};

    // Initial covariance matrices
    Q.Fill(0.0f);
    R.Fill(0.0f);
    P.Fill(0.0f);
    x.Fill(0.0f);

    // R ~ measurement noise
    R(0,0) = R(1,1) = 0.05f * 0.05f;

    // P ~ initial estimation error
    P(0,0) = P(1,1) = P(2,2) = 5.0f * 5.0f;

    // Initial Q will be filled in predict()
    
    Q(0,0) = Q(1,1) = Q(2,2) = 100.0f ;
}

void KalmanFilter::predict(const BLA::Matrix<3>& u, float dt) {
    // B = dt * identity
    B = {dt,0.0f,0.0f,
         0.0f,dt,0.0f,
         0.0f,0.0f,dt};

    // Process noise Q = B * B^T * (1°)^2 in rad
    // Q = dt^2 * identity
    Q(0,0) = Q(1,1) = Q(2,2) = (dt * 30.0f * (float)DEG_TO_RAD ) * (dt * 30.0f * (float)DEG_TO_RAD ); // 1° in rad
    x = A * x + B * u;
    P = A * P * ~A + Q;
}

void KalmanFilter::update(const BLA::Matrix<3>& accel) {
    float ax = accel(0);
    float ay = accel(1);
    float az = accel(2);

    BLA::Matrix<2> y;
    y(0) = atan2f(ay, sqrtf(ax*ax + az*az));     // roll
    y(1) = atan2f(-ax, sqrtf(ay*ay + az*az));    // pitch

    BLA::Matrix<2,2> S = H * P * ~H + R;
    bool invertible =  Invert(S);
    BLA::Matrix<3,2> K = P * ~H * S;

    x = x + K* (y - H * x);
    BLA::Matrix<3,3> I = BLA::Matrix<3,3>{1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f};
    // P = (I - K * H) * P;
    P = (I - K * H) * P;
}

BLA::Matrix<3> KalmanFilter::getEulerAnglesDeg() const{
    return x * (float)RAD_TO_DEG; // return in degrees
}
