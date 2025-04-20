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

void KalmanFilter::updateWithGnss(float course, float headingAcc)
{
    // Convert course from degrees to radians
    float yawMeasured = course * (float)DEG_TO_RAD;

    // Normalize measurement to be close to state estimate
    float yaw_est = x(2);
    while (yawMeasured - yaw_est > PI) yawMeasured -= 2.0f * PI;
    while (yawMeasured - yaw_est < -PI) yawMeasured += 2.0f * PI;

    // Construct measurement matrix for yaw (only yaw affected)
    BLA::Matrix<1,3> H_yaw = {0.0f, 0.0f, 1.0f};

    // Measurement covariance (based on heading accuracy, convert deg² → rad²)
    float R_yaw_val = headingAcc * (float)DEG_TO_RAD;
    R_yaw_val *= R_yaw_val; // variance
    BLA::Matrix<1,1> R_yaw = {R_yaw_val};

    // Innovation covariance
    BLA::Matrix<1,1> S = H_yaw * P * ~H_yaw + R_yaw;
    if (!Invert(S)) return; // Skip update if S not invertible

    // Kalman gain
    BLA::Matrix<3,1> K = P * ~H_yaw * S;

    // Measurement residual
    BLA::Matrix<1> y = {yawMeasured};
    x = x + K * (y - H_yaw * x);

    // Identity matrix
    BLA::Matrix<3,3> I = BLA::Matrix<3,3>{1.0f,0.0f,0.0f,
                                          0.0f,1.0f,0.0f,
                                          0.0f,0.0f,1.0f};

    // Update covariance
    P = (I - K * H_yaw) * P;
}



void KalmanFilter::getEulerAnglesDeg(float &roll, float &pitch, float &yaw) const {
    roll = x(0) * (float)RAD_TO_DEG;  // roll in degrees
    pitch = x(1) * (float)RAD_TO_DEG; // pitch in degrees
    yaw = x(2) * (float)RAD_TO_DEG;   // yaw in degrees
}
