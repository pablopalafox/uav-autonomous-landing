#include <ped_traj_pred/kalman_xy_pos.hpp>

using NEWMAT::Matrix;
using NEWMAT::ColumnVector;
using NEWMAT::IdentityMatrix;

KalmanXYPos::KalmanXYPos() {

    ros::NodeHandle  nh_in("~");

    nh_in.param("model_covariance", model_covariance, 2.0);
    nh_in.param("observation_covariance", observation_covariance, 1.0);

    nh_in.param("time_step", time_step, 0.5);

    state_vec_size = 4;
    observation_vec_size = 2;

    I = IdentityMatrix(state_vec_size);

    // Process error covariance
    // (measure of the estimated accuracy of the state estimate)
    P_ = Matrix(state_vec_size, state_vec_size);
    P_pred_ = Matrix(state_vec_size, state_vec_size);

    P_ = I;
    P_pred_ = I;

    // kalman gain
    K = Matrix(state_vec_size, observation_vec_size);
    K << 1 << 0
      << 0 << 1
      << 0 << 0
      << 0 << 0;

    // innovation (z - H*x)
    y = ColumnVector(observation_vec_size);
    y << 0
      << 0;

    // covariance of observation noise
    R = IdentityMatrix(observation_vec_size);
    R = R * observation_covariance;

    // observation model (which maps the true state space
    //                    into the observed space)
    H = Matrix(observation_vec_size, state_vec_size);
    H << 1 << 0 << 0 << 0
      << 0 << 1 << 0 << 0;

    // State transition model (sometimes called F)
    A = Matrix(state_vec_size, state_vec_size);
    A << 1 << 0 << time_step <<     0
      << 0 << 1 <<     0     << time_step
      << 0 << 0 <<     1     <<     0
      << 0 << 0 <<     0     <<     1;

    // Process noise covariance matrix Q
    Q = IdentityMatrix(state_vec_size);
    Q = Q * model_covariance;
    // computeProcessNoiseCovariance();

    ROS_INFO("[KALMAN XY POS] - Kalman filter init OK!");
}

KalmanXYPos::~KalmanXYPos() {}

void KalmanXYPos::computeProcessNoiseCovariance() {
    ColumnVector col_vec_G = ColumnVector(observation_vec_size);
    Matrix Q_top = Matrix(observation_vec_size, observation_vec_size);

    double delta_t_2;
    delta_t_2 = pow(time_step, 2.0);
    delta_t_2 = delta_t_2 / 2;
    col_vec_G << delta_t_2
              << time_step;

    Q_top = col_vec_G * col_vec_G.t();
    Q_top = Q_top * model_covariance;

    ROS_INFO_STREAM(Q_top(1, 1) << " " << Q_top(2, 2) << "\n");

    Q  << Q_top(1, 1) <<      0      <<      0      <<      0
       <<      0      << Q_top(2, 2) <<      0      <<      0
       <<      0      <<      0      << Q_top(1, 1) <<      0
       <<      0      <<      0      <<      0      << Q_top(2, 2);
}

void KalmanXYPos::predict() {
    P_ = A * P_ * A.t() + Q;
    if (not_moving) return;
    x_ = A * x_;
}

void KalmanXYPos::update(ColumnVector z) {
    float modZ;
    Matrix S; // innovation covariance

    modZ = (z - z0_).NormFrobenius();
    if (modZ <= 0.01) {
        y << 0
          << 0;
        not_moving = true;
    }
    else {
        y = z - H * x_; // innovation
        not_moving = false;
    }

    S = R + H * P_ * H.t();
    K = P_ * H.t() * S.i();

    z0_ = z;
}

void KalmanXYPos::correct() {
    x_ = x_ + K * y;
    P_ = (I - K * H) * P_;
}

void KalmanXYPos::predictPred() {
    P_pred_ = A * P_pred_ * A.t() + Q;
    if (not_moving) return;
    x_pred_ = A * x_pred_;
}

void KalmanXYPos::correctPred() {
    x_pred_ = x_pred_ + K * y;
    P_pred_ = (I - K * H) * P_pred_;
}

void KalmanXYPos::initializeFilter(ColumnVector z_initial)
{
    z0_ = ColumnVector(2);
    x_ = ColumnVector(4);
    x_pred_ = ColumnVector(4);

    z0_ = z_initial;
    x_  << z_initial.element(0)
        << z_initial.element(1)
        << 0
        << 0;
    x_pred_ = x_;

    ROS_INFO("[KALMAN XY POS - initializeFilter] Initial state x: %f, %f, %f, %f",
             x_.element(0), x_.element(1),
             x_.element(2), x_.element(3) );
}
