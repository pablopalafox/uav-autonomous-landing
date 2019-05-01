#ifndef PEDTRAJPRED_HPP
#define PEDTRAJPRED_HPP

#include <ros/ros.h>
#include <ped_traj_pred/predict_filter.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>



class PedTrajPred {
public:

    PedTrajPred();
    ~PedTrajPred();

    PredictFilter* predict_filter;

private:

    /// Time at previous measure
    double t_prev_measure;

    void posCallback(const geometry_msgs::PoseStampedConstPtr& pos);
    void posCovCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& PosCov);
    void odometryCallback(const nav_msgs::OdometryConstPtr& odom);

    ros::Subscriber pose_sub;
    ros::Subscriber pose_with_cov_sub;
    ros::Subscriber odometry_pose_sub;

};

#endif // PEDTRAJPRED_HPP
