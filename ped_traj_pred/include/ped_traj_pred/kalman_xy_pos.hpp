#ifndef KALMAN_XY_POS_HPP
#define KALMAN_XY_POS_HPP

/** @file kalman_xy_pos.hpp
 *  \brief Header file for the KalmanXYPos class.
 *  Details.
 */

#include <ros/ros.h>

//Lib newmat
#include <newmat/newmat.h>
//#include <newmat/newmatap.h>


/**
 * @class KalmanXYPos
 * \brief Kalman filter used for prediction of xy position
 */
class KalmanXYPos {

public:
    /**
     * Class constructor.
     */
    KalmanXYPos();
    /**
     * Class constructor, including parameters.
     */
    // KalmanXYPos(double observation_covariance, double model_covariance);

    ~KalmanXYPos();


    /// Predict step of the filter.
    void predict();

    /// Predict step, for use only when predicting future traj.
    void predictPred();

    /**
     * Updates the filter state
     * @param obs_postion Vector of observations.
     *        Provides the measurement from the sensors.
     */
    void update(NEWMAT::ColumnVector obs_position);

    /// Correction step of the filter.
    void correct();

    /// Correction step, used only when predicting future traj.
    void correctPred();

    /// Initializes the filter
    void initializeFilter(NEWMAT::ColumnVector init_position);

    /// Stores the current state of the filter
    NEWMAT::ColumnVector x_;

    /// Used for the prediction steps
    NEWMAT::ColumnVector x_pred_;

    /// Stores a previous observation.
    NEWMAT::ColumnVector z0_;

    /**
     * Covariance Matrix P.
     * x is a vector of dimension n, so P is n by n,
     * Down the diagonal of P, we find the variances of the elements of x.
     */

    NEWMAT::Matrix P_;

    /// Covariance matrix used for the prediction steps
    NEWMAT::Matrix P_pred_;

    /// whether or not the object is moving.
    bool not_moving;

    double time_step;

    /// covariance of the observation noise
    double observation_covariance;

    /// covariance of the process noise
    double model_covariance;

private:
    int state_vec_size;
    int observation_vec_size;

    // Kalman filter matrices

    /// Indentity matrix used for computation
    NEWMAT:: Matrix I;

    /// Update matrix ( A ). Models the system over time,
    /// and relates a new model state against the previuos one.
    NEWMAT::Matrix A;

    /// Covariance of the process noise
    NEWMAT::Matrix Q;

    /// Covariance of the observation noise
    NEWMAT::Matrix R;

    /// Kalman gain matrix.
    NEWMAT::Matrix K;

    /**
     * The Extraction matrix.
     * The matrix H tells us what sensor readings
     * we'd get if x were the true state of affairs and our sensors were perfect
     */
    NEWMAT::Matrix H;

    /// Difference between the observation and the kalman filter prediction.
    NEWMAT::ColumnVector y;

    void computeProcessNoiseCovariance();
};

#endif // KALMAN_XY_POS_HPP
