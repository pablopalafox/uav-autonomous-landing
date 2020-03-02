/*************************************************************************
This file is licensed under a GPLv3 License.
GPLv3 License
Copyright (C) 2016-2019 Pablo R. Palafox (pablo.rodriguez-palafox@tum.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*************************************************************************/

#include <string>
#include <boost/function.hpp>
#include <limits>
#include <fstream>
#include <sstream>
#include <math.h>
#include <cstdlib>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>

#include <cvg_sim_msgs/Altimeter.h>
#include <ardrone_autonomy/Navdata.h>

#include <takeoff/GroundtruthAltitude.h>

#include <opencv2/highgui/highgui.hpp>

#include <ped_traj_pred/PathWithId.h> // msg type for the path of predicted positions

// navigation states from the gazebo plugin (used only to send correct state to rotors in plugin)
#define UNKNOWN_MODEL       0
#define INITIALIZE_MODEL    1
#define LANDED_MODEL        2
#define FLYING_MODEL        3
#define HOVERING_MODEL      4
#define TESTING_MODEL       5
#define TAKINGOFF_MODEL     6
#define TO_FIX_POINT_MODEL  7
#define LANDING_MODEL       8
#define LOOPING_MODEL       9

// the states we actually use for our own control
namespace Status_n {
enum Status_t {
    LANDED,
    TAKINGOFF,
    TRACKING,
    RELOCALIZING,
    LANDING,
    FORCED_LANDING
};
}
typedef Status_n::Status_t Status;

constexpr int MAX_NUM_SEQUENCES = 50;

class PlatformTracking {
private:
    bool verbose_;
    bool save_logs_;
    bool manual_gains_;
    double cmd_vel_pub_freq_;
    bool adapt_path_idx_, adapt_path_idx_again_;
    bool has_taken_off_once_;

    double MISSION_DURATION;

    bool use_prediction_; // if true, use Kalman filter for predicting future positions of platform
    int use_prediction_trackbar_; // to be able to set use_prediction_ on the trackbar
    bool valid_dist_to_axis_;
    int completed_sequences_; // keep track of the number of successfull landings
    double landed_time_;
    int num_reloc_maneuvers_;

    bool should_take_off_, should_land_, must_land_;
    Status current_status_;
    double ardrone_x_, ardrone_y_, ardrone_z_;
    double sonar_range_, altitude_altimeter_;
    unsigned int navi_state_gazebo_;

    // PID auxiliary variables
    double error_sum_x_, error_sum_y_;
    double derror_x_, derror_y_;
    tf::Vector3 target_;
    tf::Vector3 target_prev_;

    // PID gains
    double Kp_, Ki_, Kd_;
    double Kp_min_, Ki_min_, Kd_min_;
    double exp_coeff_Kp_, exp_coeff_Ki_, exp_coeff_Kd_;
    int Kp_trackbar_, Ki_trackbar_, Kd_trackbar_;


    // time auxiliry variables
    bool seen_centroid_lately_;
    double current_time_;
    double t_ref_;
    double t_sequence_;
    double dt_;
    double last_found_time_;
    double time_without_seeing_;
    double MAX_ALLOWED_TIME_WITHOUT_SEEING_;
    double MAX_ALLOWED_ERROR_BEFORE_LANDING_;

    // logging
    std::ofstream errorsFile_, trajFile_;

    // messages and services
    geometry_msgs::Twist cmd_vel_; //! cmd_vel_ocity command send to ardrone for control
    geometry_msgs::Twist cmd_vel_prev_; //! we use it to check if we should publish a new vel
    std_msgs::Empty emptyMsg_;
    std_srvs::Empty emptySrv_;

    // geometry_msgs::PoseStamped current_platform_in_ardrone_; //! centroid's position wrt the quadrotor's base_link frame
    // geometry_msgs::PoseStamped current_centroid_in_world; //! centroid's position wrt world frame

    // topics
    std::string cmd_vel_topic_;
    // std::string platform_position_in_ardrone_topic_;  //! published by platform_detection node
    // std::string indicator_position_in_ardrone_topic_; //! published by platform_detection node
    std::string predicted_platform_path_in_world_topic_; //! published by ped_traj_pred

    std::string sonar_height_topic_, altitude_altimeter_topic_;
    std::string gt_altitude_topic_;
    std::string ardrone_navdata_;

    double TRACKING_ALTITUDE_;
    int TRACKING_ALTITUDE_TRACKBAR_;
    double MARGIN_TRACKING_ALTITUDE_;

    // publishers
    ros::Publisher cmd_vel_pub_; //! publishes into cmd_vel_topic_ for ardrone
    ros::Publisher navdata_pub_;

    // subscribers
    ros::Subscriber platform_position_in_ardrone_sub_;
    ros::Subscriber indicator_postition_in_ardrone_sub_;
    ros::Subscriber pred_platform_path_in_world_sub_;

    ros::Subscriber sonar_height_sub_, altitude_altimeter_sub_;
    ros::Subscriber gt_altitude_sub_;
    ros::Subscriber takeoff_sub_, land_sub_, force_land_sub_;
    ros::Subscriber ardrone_gt_sub_, summit_gt_sub_;
    ros::Subscriber ardrone_imu_sub_;
    ros::Timer timer_;

    // transformations
    tf::TransformListener tf_listener_;
    tf::StampedTransform T_ardrone_world_; // odom is our inertial frame here

    // 3d current positions of the platform's centroid in ardrone
    tf::Vector3 centroid_in_ardrone_, centroid_in_world_;
    tf::Vector3 indicator_in_ardrone_;
    double centroid_in_ardrone_stamp_;
    double indicator_in_ardrone_stamp_;

    // 3d predicted positions of the platform's centroid
    int idx_in_path_; // idx of predicted path
    int num_pos_in_path_; // number of positions in path
    tf::Vector3 pred_centroid_in_world_;
    tf::Vector3 pred_centroid_in_ardrone_;

    // groundtruth positions
    tf::Vector3 gt_ardrone_;
    tf::Vector3 gt_summit_;

    // ardrone IMU data
    double angle_x_, angle_y_;
    double angular_velocity_x_, angular_velocity_y_;
    double linear_acceleration_x_, linear_acceleration_y_;

    // auxiliary for computing distance from ardrone to platform's x and y axes
    tf::Vector3 segment_;
    tf::Vector3 perp_segment_;

    double distance_to_x_axis_platform_; //! distance from the quadrotor to the platform's x axis
    double distance_to_y_axis_platform_; //! distance from the quadrotor to the platform's y axis
    double dist_in_xy_plane_to_centroid_;

    // some helper methods
    void setErrorSumToZero(const std::string &axis);
    void setErrorDerivativeToZero();
    void setDistancesToNaN();
    void setDistancesToZero();
    void setCmdVelToZero();
    void setLandedConfig();
    void setTakingoffConfig();

    void computeDistanceToPlatformAxes();
    void computeDistanceInXYPlaneToPlatformCentroid();
    void print_status();
    void print_navdata_state();
    void relocalizationManeuver();

    bool areAllCmdVelZero();
    bool is_position_NaN(const tf::Vector3 &v);

    void saveStateToCsv(const double t,
                        const double ex_real, const double ey_real, const double ez_real,
                        const double ex_target, const double ey_target, const double ez_target);
public:
    PlatformTracking();
    ~PlatformTracking();

    void sonarCallback(const sensor_msgs::RangeConstPtr& sonar_height_quadrotor);
    void altimeterCallback(const cvg_sim_msgs::AltimeterConstPtr& altimeter_msg);
    void groundtruthAltitudeCallback(const takeoff::GroundtruthAltitudeConstPtr& gt_altitude_msg);

    void predPlatformPathCallback(const ped_traj_pred::PathWithId& predicted_path);

    void takeoffCallback(const std_msgs::EmptyConstPtr &takeoff_signal);
    void landCallback(const std_msgs::EmptyConstPtr &landing_signal);
    void forceLandCallback(const std_msgs::EmptyConstPtr &force_landing_signal);
    void heightControlCallback(const ros::TimerEvent& e);

    void groundtruthArdroneCallback(const geometry_msgs::PoseStamped& gt_ardrone);
    void groundtruthSummitCallback(const geometry_msgs::PoseStamped& gt_summit);
    void ardroneImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);


    void follow_platform();
};

PlatformTracking::PlatformTracking() {

    ros::NodeHandle nh_("~");

    verbose_ = true;
    save_logs_ = true;
    manual_gains_ = false;
    cmd_vel_pub_freq_ = 0.5;
    adapt_path_idx_ = true;
    adapt_path_idx_again_ = true;
    has_taken_off_once_ = false;

    should_take_off_ = should_land_ = must_land_ = false;

    completed_sequences_ = 0;
    landed_time_ = std::numeric_limits<double>::infinity();
    num_reloc_maneuvers_ = 0;

    idx_in_path_ = -1; // take last prediction in path
    num_pos_in_path_ = -1;

    current_status_ = Status_n::LANDED; // our own internal finite state machine
    navi_state_gazebo_ = LANDED_MODEL; // state for the rotor's plugin

    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setCmdVelToZero();

    last_found_time_ = std::numeric_limits<double>::quiet_NaN();
    time_without_seeing_ = std::numeric_limits<double>::quiet_NaN();
    t_ref_ = std::numeric_limits<double>::quiet_NaN();
    t_sequence_ = std::numeric_limits<double>::quiet_NaN();
    seen_centroid_lately_ = false;

    valid_dist_to_axis_ = false;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // get params
    nh_.getParam("verbose", verbose_);
    nh_.getParam("save_logs", save_logs_);
    nh_.getParam("manual_gains", manual_gains_);
    nh_.getParam("cmd_vel_pub_freq", cmd_vel_pub_freq_);
    nh_.getParam("adapt_path_idx", adapt_path_idx_);
    nh_.getParam("MISSION_DURATION", MISSION_DURATION);
    ROS_INFO("Mission duration %f", MISSION_DURATION);

    nh_.getParam("cmd_vel_topic", cmd_vel_topic_);
    nh_.getParam("predicted_platform_path_in_world_topic", predicted_platform_path_in_world_topic_);

    nh_.getParam("sonar_topic", sonar_height_topic_);
    nh_.getParam("altimeter_topic", altitude_altimeter_topic_);
    nh_.getParam("gt_altitude_topic", gt_altitude_topic_);
    nh_.getParam("ardrone_navdata", ardrone_navdata_);

    nh_.getParam("MAX_ALLOWED_TIME_WITHOUT_SEEING", MAX_ALLOWED_TIME_WITHOUT_SEEING_);
    nh_.getParam("MAX_ALLOWED_ERROR_BEFORE_LANDING", MAX_ALLOWED_ERROR_BEFORE_LANDING_);


    nh_.getParam("TRACKING_ALTITUDE", TRACKING_ALTITUDE_);
    ROS_INFO("TRACKING_ALTITUDE_ %f", TRACKING_ALTITUDE_);
    TRACKING_ALTITUDE_TRACKBAR_ = TRACKING_ALTITUDE_ * 100.0;
    nh_.getParam("MARGIN_TRACKING_ALTITUDE", MARGIN_TRACKING_ALTITUDE_);

    nh_.getParam("Kp", Kp_);
    nh_.getParam("Ki", Ki_);
    nh_.getParam("Kd", Kd_);

    nh_.getParam("Kp_min", Kp_min_);
    nh_.getParam("Ki_min", Ki_min_);
    nh_.getParam("Kd_min", Kd_min_);

    nh_.getParam("exp_coeff_Kp", exp_coeff_Kp_);
    nh_.getParam("exp_coeff_Ki", exp_coeff_Ki_);
    nh_.getParam("exp_coeff_Kd", exp_coeff_Kd_);

    Kp_trackbar_ = Kp_ * 1000;
    Ki_trackbar_ = Ki_ * 10000;
    Kd_trackbar_ = Kd_ * 10000;
    ROS_INFO("Kp %f, Ki %f, Kd %f", Kp_, Ki_, Kd_);

    use_prediction_ = false;
    nh_.getParam("use_prediction_", use_prediction_);
    ROS_INFO_STREAM(use_prediction_);
    use_prediction_trackbar_ = use_prediction_;
    ROS_INFO_STREAM(use_prediction_trackbar_);

    if (manual_gains_) {
        cv::namedWindow("Gains");
        cv::createTrackbar("Kp", "Gains", &Kp_trackbar_, 1000, NULL);
        cv::createTrackbar("Ki", "Gains", &Ki_trackbar_, 1000, NULL);
        cv::createTrackbar("Kd", "Gains", &Kd_trackbar_, 1000, NULL);
        cv::createTrackbar("h_track", "Gains", &TRACKING_ALTITUDE_TRACKBAR_, 600, NULL);
        cv::createTrackbar("use_pred", "Gains", &use_prediction_trackbar_, 1, NULL);
        // cv::createTrackbar("idx", "Gains", &idx_in_path_, 5, NULL);
    }
    else {
        cv::namedWindow("Params");
        cv::createTrackbar("h_track", "Params", &TRACKING_ALTITUDE_TRACKBAR_, 600, NULL);
        cv::createTrackbar("use_pred", "Params", &use_prediction_trackbar_, 1, NULL);
        // cv::createTrackbar("idx", "Params", &idx_in_path_, 5, NULL);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // publishers
    cmd_vel_pub_             = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    navdata_pub_             = nh_.advertise<ardrone_autonomy::Navdata>(ardrone_navdata_, 1);

    // subscribers
    pred_platform_path_in_world_sub_ = nh_.subscribe(predicted_platform_path_in_world_topic_, 1,
                                       &PlatformTracking::predPlatformPathCallback, this);

    sonar_height_sub_        = nh_.subscribe(sonar_height_topic_, 1, &PlatformTracking::sonarCallback, this);
    altitude_altimeter_sub_  = nh_.subscribe(altitude_altimeter_topic_, 1, &PlatformTracking::altimeterCallback, this);
    gt_altitude_sub_         = nh_.subscribe(gt_altitude_topic_, 1,
                               &PlatformTracking::groundtruthAltitudeCallback, this);

    takeoff_sub_             = nh_.subscribe("/ardrone/takeoff", 1, &PlatformTracking::takeoffCallback, this);
    land_sub_                = nh_.subscribe("/ardrone/land", 1, &PlatformTracking::landCallback, this);
    force_land_sub_          = nh_.subscribe("/ardrone/force_land", 1, &PlatformTracking::forceLandCallback, this);

    ardrone_gt_sub_          = nh_.subscribe("/groundtruth/ardrone", 1, &PlatformTracking::groundtruthArdroneCallback, this);
    summit_gt_sub_           = nh_.subscribe("/groundtruth/summit", 1, &PlatformTracking::groundtruthSummitCallback, this);
    ardrone_imu_sub_         = nh_.subscribe("/ardrone/imu", 1, &PlatformTracking::ardroneImuCallback, this);

    ROS_INFO("%f", 1.0 / cmd_vel_pub_freq_);
    timer_                   = nh_.createTimer(ros::Duration(1.0 / cmd_vel_pub_freq_), &PlatformTracking::heightControlCallback, this);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Transform from ardrone (base_link) to odom (acts like the world frame, in this case)
    try {
        ros::Time now = ros::Time::now();
        tf_listener_.waitForTransform("/ardrone/base_link", "/odom",
                                      now, ros::Duration(3.0));
        tf_listener_.lookupTransform("/ardrone/base_link", "/odom",
                                     now, T_ardrone_world_);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

} // end del constructor

PlatformTracking::~PlatformTracking() {
    ROS_INFO("Killing tracking...");
}

/////////////////////////////////////// callbacks ///////////////////////////////////////////

void PlatformTracking::takeoffCallback(const std_msgs::EmptyConstPtr & takeoff_signal) {
    t_ref_ = ros::Time::now().toSec();
    t_sequence_ = ros::Time::now().toSec();

    if (save_logs_) {
        std::string log_dir = "/home/pablo/ws/log";

        std::string errors_dir = log_dir + "/errors";
        std::string create_errors_dir = "mkdir -p " + errors_dir;

        std::string trajectories_dir = log_dir + "/trajectories";
        std::string create_trajectories_dir = "mkdir -p " + trajectories_dir;

        // Create dirs
        const int dir_errors = system(create_errors_dir.c_str());
        const int dir_trajectories = system(create_trajectories_dir.c_str());
        if (dir_errors == -1 || dir_trajectories == -1) {
            ROS_ERROR("Error creating directories");
            exit(1);
        }

        // Build file's names depending on type of approach
        std::string type;
        if (use_prediction_)
            type = "pred";
        else
            type = "NO_pred";

        std::ostringstream oss;
        oss << errors_dir << "/errors_" << type << ".csv";
        if (!errorsFile_.is_open()) {
            errorsFile_.open(oss.str());
            errorsFile_ << "t,ex_real,ey_real,ez_real,ex_target,ey_target,ez_target\n";
        }

        oss.str("");
        oss.clear();
        oss << "/home/pablo/ws/log/trajectories/trajectories_" << type << ".csv";
        if (!trajFile_.is_open()) {
            trajFile_.open(oss.str());
            trajFile_ << "status,aX,aY,aZ,sX,sY,sZ\n";
        }
    }

    should_take_off_ = true;
    // ROS_INFO("Quadrotor taking off...");
    has_taken_off_once_ = true;
}

void PlatformTracking::landCallback(const std_msgs::EmptyConstPtr & landing_signal) {
    should_land_ = true;
    ROS_INFO("Quadrotor landing...");
}

void PlatformTracking::forceLandCallback(const std_msgs::EmptyConstPtr &force_landing_signal) {
    must_land_ = true;
    ROS_INFO("Forcing landing");
}

void PlatformTracking::sonarCallback(const sensor_msgs::RangeConstPtr & sonar_msg) {
    sonar_range_ = sonar_msg->range;
    // ROS_INFO("sonar: %f\n", sonar_range_);
}

void PlatformTracking::altimeterCallback(const cvg_sim_msgs::AltimeterConstPtr & altimeter_msg) {
    altitude_altimeter_ = altimeter_msg->altitude * 100;
    // ROS_INFO("altimeter: %f\n", altitude_altimeter_);
}

void PlatformTracking::groundtruthAltitudeCallback(const takeoff::GroundtruthAltitudeConstPtr& gt_altitude_msg) {
    ardrone_z_ = gt_altitude_msg->gt_altitude;;
    // ROS_INFO("gt: %f\n", quadrotor_groundtruth_z_);
}

void PlatformTracking::groundtruthArdroneCallback(const geometry_msgs::PoseStamped& gt_ardrone) {
    gt_ardrone_.setX(gt_ardrone.pose.position.x);
    gt_ardrone_.setY(gt_ardrone.pose.position.y);
    gt_ardrone_.setZ(gt_ardrone.pose.position.z);
}

void PlatformTracking::groundtruthSummitCallback(const geometry_msgs::PoseStamped& gt_summit) {
    gt_summit_.setX(gt_summit.pose.position.x);
    gt_summit_.setY(gt_summit.pose.position.y);
    gt_summit_.setZ(gt_summit.pose.position.z);
}

void PlatformTracking::ardroneImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    // angle_x_               = imu_msg->orientation.x;
    angular_velocity_x_    = imu_msg->angular_velocity.x;

    // angle_y_               = imu_msg->orientation.y;
    angular_velocity_y_    = imu_msg->angular_velocity.y;

    linear_acceleration_x_ = imu_msg->linear_acceleration.x;
    linear_acceleration_y_ = imu_msg->linear_acceleration.y;
}
/////////////////////////////////////// control  ///////////////////////////////////////////

void PlatformTracking::print_status() {
    switch (current_status_) {
    case Status_n::LANDED:
        ROS_INFO("Status LANDED");
        break;
    case Status_n::TAKINGOFF:
        ROS_INFO("Status TAKINGOFF");
        break;
    case Status_n::TRACKING:
        ROS_INFO("Status TRACKING");
        break;
    case Status_n::RELOCALIZING:
        ROS_INFO("Status RELOCALIZING");
        break;
    case Status_n::LANDING:
        ROS_INFO("Status LANDING");
        break;
    case Status_n::FORCED_LANDING:
        ROS_INFO("Status FORCED_LANDING");
        break;
    }
}

void PlatformTracking::print_navdata_state() {
    switch (navi_state_gazebo_) {
    case UNKNOWN_MODEL:
        ROS_INFO("navdata state UNKNOWN_MODEL");
        break;
    case INITIALIZE_MODEL:
        ROS_INFO("navdata state INITIALIZE_MODEL");
        break;
    case LANDED_MODEL:
        ROS_INFO("navdata state LANDED_MODEL");
        break;
    case FLYING_MODEL:
        ROS_INFO("navdata state FLYING_MODEL");
        break;
    case HOVERING_MODEL:
        ROS_INFO("navdata state HOVERING_MODEL");
        break;
    case TESTING_MODEL:
        ROS_INFO("navdata state TESTING_MODEL");
        break;
    case TAKINGOFF_MODEL:
        ROS_INFO("navdata state TAKINGOFF_MODEL");
        break;
    case TO_FIX_POINT_MODEL:
        ROS_INFO("navdata state TO_FIX_POINT_MODEL");
        break;
    case LANDING_MODEL:
        ROS_INFO("navdata state LANDING_MODEL");
        break;
    case LOOPING_MODEL:
        ROS_INFO("navdata state LOOPING_MODEL");
        break;
    }
}

void PlatformTracking::setErrorSumToZero(const std::string &axis) {
    if (axis != "both") {
        if (axis == "x")
            error_sum_x_ = 0.0;
        else if (axis == "y")
            error_sum_y_ = 0.0;
    } else {
        error_sum_x_ = 0.0;
        error_sum_y_ = 0.0;
    }

}

void PlatformTracking::setErrorDerivativeToZero() {
    derror_x_ = 0.0;
    derror_y_ = 0.0;
}

void PlatformTracking::setDistancesToNaN() {
    dist_in_xy_plane_to_centroid_ = std::numeric_limits<double>::quiet_NaN();
    distance_to_x_axis_platform_  = std::numeric_limits<double>::quiet_NaN();
    distance_to_y_axis_platform_  = std::numeric_limits<double>::quiet_NaN();
}

void PlatformTracking::setDistancesToZero() {
    dist_in_xy_plane_to_centroid_ = 0.0;
    distance_to_x_axis_platform_  = 0.0;
    distance_to_y_axis_platform_  = 0.0;
}

void PlatformTracking::setCmdVelToZero() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.z = 0.0;
}


bool PlatformTracking::areAllCmdVelZero() {
    return cmd_vel_.linear.x == 0.0 &&
           cmd_vel_.linear.y == 0.0 &&
           cmd_vel_.linear.z == 0.0 &&
           cmd_vel_.angular.z == 0.0;
}

void PlatformTracking::saveStateToCsv(const double t,
                                      const double ex_real, const double ey_real, const double ez_real,
                                      const double ex_target, const double ey_target, const double ez_target) {
    // errors file
    errorsFile_ << t << ","
                << ex_real << "," << ey_real << "," << ez_real << ","
                << ex_target << "," << ey_target << "," << ez_target << "\n" ;

    // gt position files
    trajFile_ << current_status_ << ",";
    trajFile_ << gt_ardrone_.getX() << "," << gt_ardrone_.getY() << "," << gt_ardrone_.getZ() << ",";
    trajFile_ << gt_summit_.getX() << "," << gt_summit_.getY() << "," << gt_summit_.getZ() << "\n";
}

void PlatformTracking::relocalizationManeuver() {
    // ROS_INFO("Starting relocalization maneuver...");
    current_status_ = Status_n::RELOCALIZING;
    navi_state_gazebo_ = TAKINGOFF_MODEL;
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToNaN();
    seen_centroid_lately_ = false;

    // describe whichever trajectory (typically an ascending on) to increase view point
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 2.0;
    cmd_vel_.angular.z = 0.0;
    // ROS_INFO_STREAM("cmd_vel during relocalization: " << cmd_vel_);
    ++num_reloc_maneuvers_;
}

void PlatformTracking::setLandedConfig() {
    landed_time_ = ros::Time::now().toSec();
    // ROS_INFO("Setting landed config!");
    current_status_ = Status_n::LANDED;
    navi_state_gazebo_ = LANDED_MODEL;
    setCmdVelToZero();
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToZero();
    ++completed_sequences_;
}

void PlatformTracking::setTakingoffConfig() {
    // ROS_INFO("Setting takeoff config!");
    current_status_ = Status_n::TAKINGOFF;
    navi_state_gazebo_ = TAKINGOFF_MODEL;
    setCmdVelToZero();
    cmd_vel_.linear.z = 1.0;
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToZero();
}

void PlatformTracking::heightControlCallback(const ros::TimerEvent & e) {
    /*
     * Finite State Machine control, largely dependent on the height of flight
    */

    bool is_valid_t_sequence = std::isfinite(t_sequence_);
    if ( has_taken_off_once_ && is_valid_t_sequence && ( (ros::Time::now().toSec() - t_sequence_) >= MISSION_DURATION) ) {
        should_land_ = true;
        t_sequence_ = std::numeric_limits<double>::quiet_NaN();
    }

    TRACKING_ALTITUDE_ = TRACKING_ALTITUDE_TRACKBAR_ / 100.0;
    use_prediction_ = use_prediction_trackbar_;

    if (current_status_ == Status_n::LANDED) {

        if (/*sonar_range_ > 0.6 || */should_take_off_) {
            // ROS_INFO("Let's take off!");
            setTakingoffConfig();
            should_take_off_ = false;
        }
        else if (completed_sequences_ == MAX_NUM_SEQUENCES) {

            if (has_taken_off_once_) {
                // if we landed correctly, we should have linear acceleration, since the landing platform is assumed to constantly be moving
                if (sonar_range_ < 0.1 && (fabs(linear_acceleration_x_) > 0.1 || fabs(linear_acceleration_y_) > 0.1)) {

                    if (errorsFile_.is_open()) { // if the file was already open, it means that we had already taken off before
                        // ROS_INFO("Closing errors' file...");
                        errorsFile_.close();
                    }
                    if (trajFile_.is_open()) { // if the file was already open, it means that we had already taken off before
                        // ROS_INFO("Closing gt file...");
                        trajFile_.close();
                    }
                }

                // Could be useful to make the drone take off again in case in wrongly landed on the floor
                /*if (fabs(linear_acceleration_x_) < 1e-5 && fabs(linear_acceleration_y_) < 1e-5 &&
                        fabs(angular_velocity_x_)    < 1e-5 && fabs(angular_velocity_x_)    < 1e-5) {
                    ROS_INFO("x... %f", fabs(linear_acceleration_x_));
                    ROS_INFO("y... %f", fabs(linear_acceleration_y_));
                    ROS_INFO("looks like we're not moving...");
                    should_take_off_ = true;
                }*/
            }

        } else if (ros::Time::now().toSec() - landed_time_ > 1.0) {
            should_take_off_ = true;
            t_sequence_ = ros::Time::now().toSec();
        }
    }
    else {
        if (must_land_ && current_status_ != Status_n::FORCED_LANDING) {
            // ROS_INFO("FORCED_LANDING");
            current_status_ = Status_n::FORCED_LANDING;
            cmd_vel_.linear.z = -0.3;
            must_land_ = false;
        }
        else {
            if (current_status_ == Status_n::FORCED_LANDING) {
                // ROS_INFO("Still forcing the landing...");
                if (sonar_range_ < 0.045) {
                    // ROS_INFO("Landed! Maybe not on the platform, though...");
                    setLandedConfig();
                }
            }
            else { // if we are not in a FORCED_LANDING state, do whatever fancy stuff we need to do

                // ROS_INFO("Let's do our normal workflow...");

                if (seen_centroid_lately_) {
                    // ROS_INFO("Normal computation of time_without_seeing_");
                    time_without_seeing_ = fabs(ros::Time::now().toSec() - last_found_time_);
                    // ROS_INFO("%f", time_without_seeing_);
                }
                else  {
                    // ROS_INFO("Setting time_without_seeing_ to MAX");
                    // if we have not seen the landing platform for a long time
                    time_without_seeing_ = std::numeric_limits<double>::max();

                }

                if (current_status_ != Status_n::RELOCALIZING) {

                    // ROS_INFO("We're not relocalizing, so let's actually do our normal workflow...");
                    // ROS_INFO("%f", time_without_seeing_);
                    if (time_without_seeing_ > MAX_ALLOWED_TIME_WITHOUT_SEEING_) {
                        // ROS_INFO("Damn it, we lost it...");
                        relocalizationManeuver();
                    }
                    else if (current_status_ != Status_n::LANDING) {

                        if (should_land_) {
                            // ROS_INFO("Let's land");
                            current_status_ = Status_n::LANDING;
                            // navi_state_gazebo_ = LANDING_MODEL;
                            should_land_ = false;
                        }
                        else if (current_status_ == Status_n::TAKINGOFF) {

                            // ROS_INFO("Still TAKINGOFF... - %f / %f", ardrone_z_, TRACKING_ALTITUDE_);
                            if (ardrone_z_ > TRACKING_ALTITUDE_) {
                                //  when TRACKING_ALTITUDE_ reached, automatically change to TRACKING mode
                                // ROS_INFO("TRACKING_ALTITUDE of %f reached!! Let's just track...", TRACKING_ALTITUDE_);
                                cmd_vel_.linear.z = 0.0;
                                current_status_ = Status_n::TRACKING;
                                navi_state_gazebo_ = FLYING_MODEL;
                            }

                            if (ardrone_z_ > 1.5) {
                                // ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEEE");
                                navi_state_gazebo_ = FLYING_MODEL;
                            }
                        }
                        else if (current_status_ == Status_n::TRACKING) {

                            // ROS_INFO("I'm tracking...");

                            if (ardrone_z_ < 1.5) {
                                navi_state_gazebo_ = TAKINGOFF_MODEL;
                            }
                            else {
                                navi_state_gazebo_ = FLYING_MODEL;

                                if (ardrone_z_ < (TRACKING_ALTITUDE_ - MARGIN_TRACKING_ALTITUDE_)) {
                                    cmd_vel_.linear.z = 1.0;
                                }
                                else if (ardrone_z_ > (TRACKING_ALTITUDE_ + MARGIN_TRACKING_ALTITUDE_)) {
                                    cmd_vel_.linear.z = -1.0;
                                }
                                else  {
                                    cmd_vel_.linear.z = 0.0;
                                }
                            }
                        }
                    }
                    else { // if we are landing

                        // default landing speed
                        cmd_vel_.linear.z = -0.3;

                        // ROS_INFO("We're stil LANDING...");
                        // ROS_INFO("We're stil LANDING... %f", linear_acceleration_x_);
                        // ROS_INFO("We're stil LANDING... %f", linear_acceleration_y_);
                        if (sonar_range_ < 0.1) {
                            // if sonar says we are on a surface and altitude says were are still flying
                            // then there's a hight chance we have landed successfully on the platform
                            // ROS_INFO("We landed!!!!!");
                            if (fabs(target_.getX()) > MAX_ALLOWED_ERROR_BEFORE_LANDING_
                                    || fabs(target_.getY()) > MAX_ALLOWED_ERROR_BEFORE_LANDING_) {
                                relocalizationManeuver();
                            } else {
                                setLandedConfig();
                            }
                        }
                        else if (sonar_range_ < 0.7) { // 0.7
                            if (fabs(target_.getX()) > MAX_ALLOWED_ERROR_BEFORE_LANDING_
                                    || fabs(target_.getY()) > MAX_ALLOWED_ERROR_BEFORE_LANDING_) {
                                relocalizationManeuver();
                            } else {
                                cmd_vel_.linear.z = -2.0; // 5.0
                            }
                            /*if (use_prediction_) {
                                idx_in_path_ = 0;
                            }*/
                        }
                        /*else if (sonar_range_ < 2.0) {
                            if (use_prediction_ && adapt_path_idx_again_) {
                                idx_in_path_ = 0;
                                adapt_path_idx_again_ = false;
                            }
                        }*/
                        /*else if (sonar_range_ < 3.0) {
                            if (use_prediction_ && adapt_path_idx_) {
                                idx_in_path_ = std::max(idx_in_path_ - 1, 0);
                                adapt_path_idx_ = false;
                            }
                        }*/

                    }
                }
                else {
                    // ROS_INFO("Relocalizing...");
                }
            }
        }
    }

    // height-adaptive PID gains
    if (manual_gains_) {
        Kp_ = Kp_trackbar_ / 1000.0;
        Ki_ = Ki_trackbar_ / 10000.0;
        Kd_ = Kd_trackbar_ / 10000.0;
    }
    else {
        Kp_ = Kp_min_ * exp(- exp_coeff_Kp_ * ardrone_z_);
        Ki_ = Ki_min_ * exp(- exp_coeff_Ki_ * ardrone_z_);
        Kd_ = Kd_min_ * exp(- exp_coeff_Kd_ * ardrone_z_);
    }

    cv::waitKey(1); // to display params trackbar

    // send velocity only if not all commands are 0
    if (!areAllCmdVelZero()) {
        cmd_vel_pub_.publish(cmd_vel_);
        cmd_vel_prev_ = cmd_vel_;
    }

    // prepare navdata msg - essentially we want to send the correct state to the rotors
    ardrone_autonomy::Navdata navdata;
    navdata.state = navi_state_gazebo_;
    navdata_pub_.publish(navdata);

    if (verbose_) {
        print_status();
        print_navdata_state();
        ROS_INFO("use_prediction %d", use_prediction_);
        ROS_INFO("pred path idx %d", idx_in_path_);
        ROS_INFO("groundtruth ardrone_z %f", ardrone_z_);
        ROS_INFO("sonar %f", sonar_range_);
        // ROS_INFO("altimeter %f", altitude_altimeter_);
        ROS_INFO("TRACKING_ALTITUDE %f", TRACKING_ALTITUDE_);
        ROS_INFO_STREAM("Seen centroid lately: " << seen_centroid_lately_);
        // ROS_INFO_STREAM("Last found time: " << last_found_time_);
        // ROS_INFO_STREAM("dt: " << dt_);
        // ROS_INFO_STREAM("Time w/o seeing: " << time_without_seeing_);
        // ROS_INFO_STREAM("Dist in xy plane to centroid: " << dist_in_xy_plane_to_centroid_);
        // ROS_INFO_STREAM("Dist to platform's X axis: " << distance_to_x_axis_platform_);
        // ROS_INFO_STREAM("Dist to platform's Y axis: " << distance_to_y_axis_platform_);
        ROS_INFO("Derivative (%f, %f)", derror_x_, derror_y_);
        ROS_INFO("Error sum (%f, %f)", error_sum_x_, error_sum_y_);
        ROS_INFO("Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);
        ROS_INFO("ex: %f, ey: %f, ez: %f",
                 target_.getX(),
                 target_.getY(),
                 target_.getZ());
        ROS_INFO("linear: (%f, %f, %f)", cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z);
        ROS_INFO("yaw: %f", cmd_vel_.angular.z);
        ROS_INFO("completed maneuvers %d", completed_sequences_);
        ROS_INFO("reloc maneuvers %d", num_reloc_maneuvers_);
        ROS_INFO("---");
    }

    /// save errors to file
    if (errorsFile_.is_open()) {
        saveStateToCsv(current_time_ - t_ref_,
                       centroid_in_ardrone_.getX(),
                       centroid_in_ardrone_.getY(),
                       centroid_in_ardrone_.getZ(),
                       target_.getX(),
                       target_.getY(),
                       target_.getZ());
    }
}

bool PlatformTracking::is_position_NaN(const tf::Vector3 &v) {
    return std::isnan(v.getX()) || std::isnan(v.getY()) || std::isnan(v.getZ());
}

void PlatformTracking::computeDistanceInXYPlaneToPlatformCentroid() {
    dist_in_xy_plane_to_centroid_ = sqrt(pow(centroid_in_ardrone_.getX(), 2) + pow(centroid_in_ardrone_.getY(), 2));
}

void PlatformTracking::predPlatformPathCallback(const ped_traj_pred::PathWithId& predicted_path) {

    num_pos_in_path_ = predicted_path.path.poses.size();

    if (idx_in_path_ >= num_pos_in_path_) {
        ROS_FATAL("Requested idx %d of path, but max idx is %d", idx_in_path_, num_pos_in_path_ - 1);
        ROS_INFO("Falling back to idx %d", num_pos_in_path_ - 1);
        idx_in_path_ = num_pos_in_path_ - 1;
    }
    else if (idx_in_path_ == -1) {
        // get last prediction in predicted path
        idx_in_path_ = 1 /*num_pos_in_path_ - 1*/;
    }

    // predicted ardrone pose in world
    geometry_msgs::PoseStamped pred_pose_in_world = predicted_path.path.poses[idx_in_path_];
    pred_centroid_in_world_ = tf::Vector3(pred_pose_in_world.pose.position.x,
                                          pred_pose_in_world.pose.position.y,
                                          pred_pose_in_world.pose.position.z);

    // current ardrone pose in world
    geometry_msgs::PoseStamped pose_in_world = predicted_path.path.poses[0];
    centroid_in_world_ = tf::Vector3(pose_in_world.pose.position.x,
                                     pose_in_world.pose.position.y,
                                     pose_in_world.pose.position.z);

    if (is_position_NaN(pred_centroid_in_world_)) {
        ROS_INFO("pred_centroid_in_world_ is NaN!");
        return;
    }

    if (is_position_NaN(centroid_in_world_)) {
        ROS_INFO("centroid_in_world_ is NaN!");
        return;
    }

    try {
        tf_listener_.lookupTransform("/ardrone/base_link", "/odom",
                                     ros::Time(0), T_ardrone_world_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    pred_centroid_in_ardrone_ = T_ardrone_world_ * pred_centroid_in_world_;
    centroid_in_ardrone_ = T_ardrone_world_ * centroid_in_world_;

    if (current_status_ == Status_n::RELOCALIZING) {
        ROS_INFO("predPlatformPathCallback::Platform found again!!!");
        ROS_INFO("predPlatformPathCallback::Tracking...");
        // if we were relocalizing the landing platform, stop the reloc maneuver
        setCmdVelToZero();
        // and set the status to TRACKING
        current_status_ = Status_n::TRACKING;
        t_sequence_ = ros::Time::now().toSec();
    }

    follow_platform();
}

void PlatformTracking::follow_platform() {

    /// our target is different depending on whether we use prediction or not
    if (use_prediction_) {
        target_ = pred_centroid_in_ardrone_;
    }
    else {
        target_ = centroid_in_ardrone_;
    }

    // deal with the time increments
    current_time_ = ros::Time::now().toSec();

    // initializing the system and exiting the function the first time
    if (!seen_centroid_lately_) {
        ROS_INFO("Re-seen!!");
        last_found_time_ = current_time_;
        seen_centroid_lately_ = true;
        target_prev_ = target_;
        return;
    }
    else if (seen_centroid_lately_ && current_status_ == Status_n::LANDED) {
        last_found_time_ = current_time_;
    }

    if (current_status_ != Status_n::LANDED && current_status_ != Status_n::FORCED_LANDING) {

        /// delta t (time increment)
        dt_ =  current_time_ - last_found_time_;
        if (dt_ < 1E-10)
            return;

        if (std::isnan(dt_)) {
            setCmdVelToZero();
            setErrorSumToZero("both");
            setErrorDerivativeToZero();
            return;
        }

        // Integral action
        error_sum_x_ += (target_.getX() * dt_);
        error_sum_y_ += (target_.getY() * dt_);

        // Derivative action
        derror_x_ = (target_.getX() - target_prev_.getX()) / dt_;
        derror_y_ = (target_.getY() - target_prev_.getY()) / dt_;

        computeDistanceInXYPlaneToPlatformCentroid();

        if (dist_in_xy_plane_to_centroid_ <= 0.01) {
            // if really close to the target, better stay still
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
        }
        // else if (dist_in_xy_plane_to_centroid_ <= 0.40) {
        //     // if close to the target, use full PID
        //     cmd_vel_.linear.x = (Kp_ * target_.getX()) + (Ki_ * error_sum_x_) + (Kd_ * derror_x_);
        //     cmd_vel_.linear.y = (Kp_ * target_.getY()) + (Ki_ * error_sum_y_) + (Kd_ * derror_y_);
        // }
        else {
            cmd_vel_.linear.x = (Kp_ * target_.getX()) + (Ki_ * error_sum_x_) + (Kd_ * derror_x_);
            cmd_vel_.linear.y = (Kp_ * target_.getY()) + (Ki_ * error_sum_y_) + (Kd_ * derror_y_);
        }

        // store target and current time for next iteration
        target_prev_ = target_;
        last_found_time_ = current_time_;
    }
} // end of tracking callback

int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_tracking");
    PlatformTracking platform_tracking;
    ROS_INFO("Set up completed. Ready to start mission");
    ros::spin();
}

