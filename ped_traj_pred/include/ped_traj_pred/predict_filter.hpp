#ifndef PREDICTFILTER_H
#define PREDICTFILTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ped_traj_pred/kalman_xy_pos.hpp>

//Position messages
#include <visualization_msgs/Marker.h>

// Path with id to RiskRRT planner
#include <ped_traj_pred/PathWithId.h> //Path with id

class PredictFilter : KalmanXYPos {
public:
    PredictFilter();
    ~PredictFilter();

    // void publishSpin(const ros::TimerEvent& e);
    void publishPred(float pos_x, float pos_y, float pos_z);
    void refreshPos(float pos_x, float pos_y);

    bool filter_initialized;
    std::string output_frame_id;

    double getTimeStep() const;
    void setTimeStep(double value);

    ros::Timer publish_timer;

    int getPathId() const;
    void setPathId(int value);

private:

    int num_pos;

    int path_id;

    ros::Publisher  visualization_pub;
    ros::Publisher  future_pos_pub;
    ros::Publisher  future_path_pub;

    ////////////////////////////////////////////////////
    visualization_msgs::Marker points, line_strip;
    geometry_msgs::Point mark; //auxiliary variable for viz
    geometry_msgs::PoseWithCovarianceStamped poswithcov;
};

#endif // PREDICTFILTER_H
