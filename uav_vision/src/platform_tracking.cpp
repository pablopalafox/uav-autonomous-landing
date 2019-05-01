#include <string>
#include <boost/function.hpp>
#include <limits>
#include <fstream>
#include <sstream>
#include <math.h>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Range.h>

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

constexpr double MAX_ALLOWED_TIME_WITHOUT_SEEING = 2.0; // in seconds

class PlatformTracking {
private:
    bool verbose_;
    bool manual_gains_;
    double cmd_vel_pub_freq_;

    bool use_prediction_; // if true, use Kalman filter for predicting future positions of platform
    int use_prediction_trackbar_; // to be able to set use_prediction_ on the trackbar
    bool valid_dist_to_axis_;
    int landings_count_; // keep track of the number of successfull landings

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
    double t_ref;
    double dt_;
    double last_found_time_;
    double time_without_seeing_;

    // logging
    std::ofstream errorsFile_;

    // messages and services
    geometry_msgs::Twist cmd_vel_; //! cmd_vel_ocity command send to ardrone for control
    geometry_msgs::Twist cmd_vel_prev_; //! we use it to check if we should publish a new vel
    std_msgs::Empty emptyMsg_;
    std_srvs::Empty emptySrv_;

    // geometry_msgs::PoseStamped current_platform_in_ardrone_; //! centroid's position wrt the quadrotor's base_link frame
    // geometry_msgs::PoseStamped current_centroid_in_world; //! centroid's position wrt world frame

    // topics
    std::string cmd_vel_topic_;
    std::string platform_position_in_ardrone_topic_;  //! published by platform_detection node
    std::string indicator_position_in_ardrone_topic_; //! published by platform_detection node
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
    ros::Timer timer_;

    // transformations
    tf::TransformListener tf_listener_;
    tf::StampedTransform T_ardrone_world_; // odom is our inertial frame here

    // 3d current positions of the platform's centroid in ardrone
    tf::Vector3 centroid_in_ardrone_;
    tf::Vector3 indicator_in_ardrone_;
    double centroid_in_ardrone_stamp_;
    double indicator_in_ardrone_stamp_;

    // 3d predicted positions of the platform's centroid
    int idx_in_path_; // idx of predicted path
    tf::Vector3 pred_centroid_in_world_;
    tf::Vector3 pred_centroid_in_ardrone_;
    double pred_path_in_world_stamp_;
    double pred_centroid_in_world_stamp_;

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

    void saveErrorsToCsv(const double t, const double ex, const double ey, const double ez);
    std::string getErrorsFileName();

public:
    PlatformTracking();
    ~PlatformTracking();

    void sonarCallback(const sensor_msgs::RangeConstPtr& sonar_height_quadrotor);
    void altimeterCallback(const cvg_sim_msgs::AltimeterConstPtr& altimeter_msg);
    void groundtruthAltitudeCallback(const takeoff::GroundtruthAltitudeConstPtr& gt_altitude_msg);

    void centroidPositionCallback(const geometry_msgs::PoseStamped& current_centroid);
    void indicatorPositionCallback(const geometry_msgs::PoseStamped& current_indicator);
    void predPlatformPathCallback(const ped_traj_pred::PathWithId& predicted_path);

    void takeoffCallback(const std_msgs::EmptyConstPtr &takeoff_signal);
    void landCallback(const std_msgs::EmptyConstPtr &landing_signal);
    void forceLandCallback(const std_msgs::EmptyConstPtr &force_landing_signal);
    void heightControlCallback(const ros::TimerEvent& e);

    void follow_platform(bool from_predicted);
};

PlatformTracking::PlatformTracking() {

    ros::NodeHandle nh_("~");

    verbose_ = true;
    manual_gains_ = false;
    cmd_vel_pub_freq_ = 0.5;

    should_take_off_ = should_land_ = must_land_ = false;

    landings_count_ = 0;

    idx_in_path_ = 0; // first prediction in the path of prediction

    current_status_ = Status_n::LANDED; // our own internal finite state machine
    navi_state_gazebo_ = LANDED_MODEL; // state for the rotor's plugin

    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setCmdVelToZero();

    t_ref = 0.0;
    last_found_time_ = std::numeric_limits<double>::quiet_NaN();
    time_without_seeing_ = std::numeric_limits<double>::quiet_NaN();
    seen_centroid_lately_ = false;

    valid_dist_to_axis_ = false;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // get params
    nh_.getParam("verbose", verbose_);
    nh_.getParam("manual_gains", manual_gains_);
    nh_.getParam("cmd_vel_pub_freq", cmd_vel_pub_freq_);

    nh_.getParam("cmd_vel_topic", cmd_vel_topic_);
    nh_.getParam("platform_position_in_ardrone_topic", platform_position_in_ardrone_topic_);
    nh_.getParam("indicator_position_in_ardrone_topic", indicator_position_in_ardrone_topic_);
    nh_.getParam("predicted_platform_path_in_world_topic", predicted_platform_path_in_world_topic_);

    nh_.getParam("sonar_topic", sonar_height_topic_);
    nh_.getParam("altimeter_topic", altitude_altimeter_topic_);
    nh_.getParam("gt_altitude_topic", gt_altitude_topic_);
    nh_.getParam("ardrone_navdata", ardrone_navdata_);

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

    if (manual_gains_) {
        cv::namedWindow("Gains");
        cv::createTrackbar("Kp", "Gains", &Kp_trackbar_, 1000, NULL);
        cv::createTrackbar("Ki", "Gains", &Ki_trackbar_, 1000, NULL);
        cv::createTrackbar("Kd", "Gains", &Kd_trackbar_, 1000, NULL);
        cv::createTrackbar("h_track", "Gains", &TRACKING_ALTITUDE_TRACKBAR_, 600, NULL);
        cv::createTrackbar("use_pred", "Gains", &use_prediction_trackbar_, 1, NULL);
    }
    else {
        cv::namedWindow("Params");
        cv::createTrackbar("h_track", "Params", &TRACKING_ALTITUDE_TRACKBAR_, 600, NULL);
        cv::createTrackbar("use_pred", "Params", &use_prediction_trackbar_, 1, NULL);
    }


    use_prediction_ = false;
    nh_.getParam("use_prediction_", use_prediction_);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // publishers
    cmd_vel_pub_             = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    navdata_pub_             = nh_.advertise<ardrone_autonomy::Navdata>(ardrone_navdata_, 1);

    // subscribers
    platform_position_in_ardrone_sub_   = nh_.subscribe(platform_position_in_ardrone_topic_, 1,
                                          &PlatformTracking::centroidPositionCallback, this);
    indicator_postition_in_ardrone_sub_ = nh_.subscribe(indicator_position_in_ardrone_topic_, 1,
                                          &PlatformTracking::indicatorPositionCallback, this);
    pred_platform_path_in_world_sub_    = nh_.subscribe(predicted_platform_path_in_world_topic_, 1,
                                          &PlatformTracking::predPlatformPathCallback, this);

    sonar_height_sub_        = nh_.subscribe(sonar_height_topic_, 1, &PlatformTracking::sonarCallback, this);
    altitude_altimeter_sub_  = nh_.subscribe(altitude_altimeter_topic_, 1, &PlatformTracking::altimeterCallback, this);
    gt_altitude_sub_         = nh_.subscribe(gt_altitude_topic_, 1,
                               &PlatformTracking::groundtruthAltitudeCallback, this);

    takeoff_sub_             = nh_.subscribe("/ardrone/takeoff", 1, &PlatformTracking::takeoffCallback, this);
    land_sub_                = nh_.subscribe("/ardrone/land", 1, &PlatformTracking::landCallback, this);
    force_land_sub_          = nh_.subscribe("/ardrone/force_land", 1, &PlatformTracking::forceLandCallback, this);

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
    errorsFile_.open(getErrorsFileName());
    errorsFile_ << "t,ex,ey,ez\n";
    should_take_off_ = true;
    ROS_INFO("Quadrotor taking off...");
}

void PlatformTracking::landCallback(const std_msgs::EmptyConstPtr & landing_signal) {
    //current_status_ = Status_n::LANDING;
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

std::string PlatformTracking::getErrorsFileName() {
    std::ostringstream oss;
    oss << "/home/pablo/ws/log/errors/errors_test_" << landings_count_ << ".csv";
    ++landings_count_;
    return oss.str();
}

bool PlatformTracking::areAllCmdVelZero() {
    return cmd_vel_.linear.x == 0.0 &&
           cmd_vel_.linear.y == 0.0 &&
           cmd_vel_.linear.z == 0.0 &&
           cmd_vel_.angular.z == 0.0;
}

void PlatformTracking::saveErrorsToCsv(const double t, const double ex, const double ey, const double ez) {
    std::ostringstream oss;
    oss << t << "," << ex << "," << ey << "," << ez << "\n";
    errorsFile_ << oss.str();
}

void PlatformTracking::relocalizationManeuver() {
    ROS_INFO("Starting relocalization maneuver...");
    current_status_ = Status_n::RELOCALIZING;
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToNaN();
    seen_centroid_lately_ = false;

    // describe whichever trajectory (typically an ascending on) to increase view point
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.5;
    cmd_vel_.angular.z = 0.0;
    ROS_INFO_STREAM("cmd_vel during relocalization: " << cmd_vel_);
}

void PlatformTracking::setLandedConfig() {
    ROS_INFO("Setting landed config!");
    current_status_ = Status_n::LANDED;
    navi_state_gazebo_ = LANDED_MODEL;
    setCmdVelToZero();
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToZero();
}

void PlatformTracking::setTakingoffConfig() {
    ROS_INFO("Setting takeoff config!");
    current_status_ = Status_n::TAKINGOFF;
    navi_state_gazebo_ = FLYING_MODEL;
    setCmdVelToZero();
    cmd_vel_.linear.z = 0.5;
    setErrorSumToZero("both");
    setErrorDerivativeToZero();
    setDistancesToZero();
}

void PlatformTracking::heightControlCallback(const ros::TimerEvent & e) {
    /*
     * Finite State Machine control, largely dependent on the height of flight
    */

    TRACKING_ALTITUDE_ = TRACKING_ALTITUDE_TRACKBAR_ / 100.0;
    use_prediction_ = use_prediction_trackbar_;

    if (current_status_ == Status_n::LANDED ) {
        if (sonar_range_ > 0.05 || should_take_off_) { // while landed, the sonar measures 0.04
            // if the node was launched with the ardrone already in the air
            ROS_INFO("Let's take off!");
            setTakingoffConfig();
            should_take_off_ = false;
        }
        else if (errorsFile_.is_open()) { // if the file was already open, it means that we had already taken off before
            ROS_INFO("Closing errors' file...");
            errorsFile_.close();
        }
    }
    else {
        if (must_land_ && current_status_ != Status_n::FORCED_LANDING) {
            ROS_INFO("FORCED_LANDING");
            current_status_ = Status_n::FORCED_LANDING;
            cmd_vel_.linear.z = -0.2;
            must_land_ = false;
        }
        else {
            if (current_status_ == Status_n::FORCED_LANDING) {
                ROS_INFO("Still forcing the landing...");
                if (sonar_range_ < 0.045) {
                    ROS_INFO("Landed! Maybe not on the platform, though...");
                    setLandedConfig();
                }
            }
            else { // if we are not in a FORCED_LANDING state, do whatever fancy stuff we need to do

                // ROS_INFO("Let's do our normal workflow...");

                if (seen_centroid_lately_)
                    time_without_seeing_ = fabs(ros::Time::now().toSec() - last_found_time_);
                else // if we have not seen the landing platform for a long time
                    time_without_seeing_ = std::numeric_limits<double>::max();

                if (current_status_ != Status_n::RELOCALIZING) {

                    // ROS_INFO("We're not relocalizing, so let's actually do our normal workflow...");

                    if (time_without_seeing_ > MAX_ALLOWED_TIME_WITHOUT_SEEING) {
                        ROS_INFO("Damn it, we lost it...");
                        relocalizationManeuver();
                    }
                    else if (current_status_ != Status_n::LANDING) {

                        if (should_land_) {
                            ROS_INFO("Let's land");
                            current_status_ = Status_n::LANDING;
                            cmd_vel_.linear.z = -0.2;
                            should_land_ = false;
                        }
                        else if (current_status_ == Status_n::TAKINGOFF) {
                            ROS_INFO("Still TAKINGOFF...");
                            if (ardrone_z_ > TRACKING_ALTITUDE_) {
                                //  when TRACKING_ALTITUDE_ reached, automatically change to TRACKING mode
                                ROS_INFO("TRACKING_ALTITUDE of %f reached!! Let's just track...", TRACKING_ALTITUDE_);
                                cmd_vel_.linear.z = 0.0;
                                current_status_ = Status_n::TRACKING;
                            }
                        }
                        else if (current_status_ == Status_n::TRACKING) {
                            // ROS_INFO("We're tracking...");
                            // always keep the drone within TRACKING_ALTITUD_ +/- MARGIN_TRACKING_ALTITUDE_
                            // ROS_INFO_STREAM(ardrone_z_ << " | " << (TRACKING_ALTITUDE_ + MARGIN_TRACKING_ALTITUDE_));
                            if (ardrone_z_ > (TRACKING_ALTITUDE_ + MARGIN_TRACKING_ALTITUDE_))
                                cmd_vel_.linear.z = -0.2;
                            else if (ardrone_z_ < (TRACKING_ALTITUDE_ - MARGIN_TRACKING_ALTITUDE_))
                                cmd_vel_.linear.z = 0.2;
                            else
                                cmd_vel_.linear.z = 0.0;
                        }
                    }
                    else { // if we are landing
                        ROS_INFO("We're stil LANDING...");
                        if (sonar_range_ < 0.1 &&  (ardrone_z_ > 0.45 && ardrone_z_ < 0.6)) {
                            // if sonar says we are on a surface and altitude says were are still flying
                            // then there's a hight chance we have landed successfully on the platform
                            ROS_INFO("We landed!!!!!");
                            setLandedConfig();
                        }
                        else if (sonar_range_ <= 0.5 && ardrone_z_ > 0.6) {
                            ROS_INFO("Going down faster, almost there!");
                            // if sonar says we're 0.5 m away from a surface and altitude says were
                            // are 0.8 m away from the real floor, then increase descend velocity,
                            // as if we shuted down the rotors
                            cmd_vel_.linear.z = -0.6;
                        }
                        else if (sonar_range_ > 0.5) {
                            ROS_INFO("Going down faster");
                            // if sonar says we're 0.5 m away from a surface and altitude says were
                            // are 0.8 m away from the real floor, then increase descend velocity,
                            // as if we shuted down the rotors
                            cmd_vel_.linear.z = -0.2;
                        }
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
        cv::waitKey(1); // to display PID gains trackbar
    }
    else {
        Kp_ = Kp_min_ * exp(- exp_coeff_Kp_ * ardrone_z_);
        Ki_ = Ki_min_ * exp(- exp_coeff_Ki_ * ardrone_z_);
        Kd_ = Kd_min_ * exp(- exp_coeff_Kd_ * ardrone_z_);
    }

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
        ROS_INFO("groundtruth ardrone_z %f", ardrone_z_);
        ROS_INFO("sonar %f", sonar_range_);
        // ROS_INFO("altimeter %f", altitude_altimeter_);
        ROS_INFO("TRACKING_ALTITUDE %f", TRACKING_ALTITUDE_);
        ROS_INFO_STREAM("Seen centroid lately: " << seen_centroid_lately_);
        ROS_INFO_STREAM("Last found time: " << last_found_time_);
        ROS_INFO_STREAM("dt: " << dt_);
        ROS_INFO_STREAM("Time w/o seeing: " << time_without_seeing_);
        ROS_INFO_STREAM("Dist in xy plane to centroid: " << dist_in_xy_plane_to_centroid_);
        ROS_INFO_STREAM("Dist to platform's X axis: " << distance_to_x_axis_platform_);
        ROS_INFO_STREAM("Dist to platform's Y axis: " << distance_to_y_axis_platform_);
        ROS_INFO("Derivative (%f, %f)", derror_x_, derror_y_);
        ROS_INFO("Error sum (%f, %f)", error_sum_x_, error_sum_y_);
        ROS_INFO("Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);
        ROS_INFO("ex: %f, ey: %f, ez: %f",
                 centroid_in_ardrone_.getX(),
                 centroid_in_ardrone_.getY(),
                 centroid_in_ardrone_.getZ());
        if (!areAllCmdVelZero()) {
            ROS_INFO("linear: (%f, %f, %f)", cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z);
            ROS_INFO("yaw: %f", cmd_vel_.angular.z);
        }
        ROS_INFO("---");
    }
}

void PlatformTracking::computeDistanceInXYPlaneToPlatformCentroid() {
    dist_in_xy_plane_to_centroid_ = sqrt(pow(centroid_in_ardrone_.getX(), 2) + pow(centroid_in_ardrone_.getY(), 2));
}

void PlatformTracking::computeDistanceToPlatformAxes() {

    const double diff = indicator_in_ardrone_stamp_ - centroid_in_ardrone_stamp_;

    // check if the indicator and the centroid corresponding to roughly the same moment in time
    if (diff > 0.05) {
        valid_dist_to_axis_ = false;
        return;
    }

    //SEGMENT VECTOR
    segment_.setX(indicator_in_ardrone_.getX() - centroid_in_ardrone_.getX());
    segment_.setY(indicator_in_ardrone_.getY() - centroid_in_ardrone_.getY());

    //Hallamos el vector unitario de segment
    double segment_norm = sqrt( pow(segment_.getX(), 2) + pow(segment_.getY(), 2) );
    segment_.setX( segment_.getX() / segment_norm);
    segment_.setY( segment_.getY() / segment_norm);
    // ROS_INFO("segment in ardrone: (%f, %f)", segment_.getX(), segment_.getY());

    //PERP_SEGMENT VECTOR (ya va a ser perpendicular)
    perp_segment_.setX(-segment_.getY());
    perp_segment_.setY(segment_.getX());

    // ROS_INFO("perp_segment in ardrone: (%f, %f)", perp_segment_.getX(), perp_segment_.getY());

    distance_to_x_axis_platform_ = fabs((centroid_in_ardrone_.getX() * perp_segment_.getX())
                                        + (centroid_in_ardrone_.getY() * perp_segment_.getY()));

    distance_to_y_axis_platform_ = fabs((centroid_in_ardrone_.getX() * segment_.getX())
                                        + (centroid_in_ardrone_.getY() * segment_.getY()));
    valid_dist_to_axis_ = true;
}

void PlatformTracking::indicatorPositionCallback(const geometry_msgs::PoseStamped &current_indicator) {
    indicator_in_ardrone_ = tf::Vector3(current_indicator.pose.position.x,
                                        current_indicator.pose.position.y,
                                        current_indicator.pose.position.z);
    indicator_in_ardrone_stamp_  = current_indicator.header.stamp.toSec();
}

void PlatformTracking::predPlatformPathCallback(const ped_traj_pred::PathWithId& predicted_path) {


    int num_pos = predicted_path.path.poses.size();
    if (idx_in_path_ >= num_pos) {
        ROS_FATAL("Requested idx %d of path, but max idx is %d", idx_in_path_, num_pos - 1);
        ROS_INFO("Falling back to idx %d", num_pos - 1);
        idx_in_path_ = num_pos - 1;
    }

    /// retrieve the path's element define by idx_in_path_
    /// * idx_in_path_ = 0 means we get the closest prediction (less noisy, thus more accurate)
    /// * idx_in_path_ = end() means we get the latest prediction (more noisy, thus less accurate)
    /// NOTE: idx_in_path_ changes dynamically with height
    // ROS_INFO("Getting position %d from predicted path", idx_in_path_);
    geometry_msgs::PoseStamped pred_pose_in_world = predicted_path.path.poses[idx_in_path_];

    pred_centroid_in_world_ = tf::Vector3(pred_pose_in_world.pose.position.x,
                                          pred_pose_in_world.pose.position.y,
                                          pred_pose_in_world.pose.position.z);

    pred_path_in_world_stamp_ = predicted_path.path.header.stamp.toSec();
    pred_centroid_in_world_stamp_ = pred_pose_in_world.header.stamp.toSec(); // TODO: do we need this one?

    try {
        tf_listener_.waitForTransform("/ardrone/base_link", "/odom",
                                      predicted_path.path.header.stamp, ros::Duration(3.0));
        tf_listener_.lookupTransform("/ardrone/base_link", "/odom",
                                     predicted_path.path.header.stamp, T_ardrone_world_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    pred_centroid_in_ardrone_ = T_ardrone_world_ * pred_centroid_in_world_;

    ROS_INFO("%f %f %f", pred_centroid_in_ardrone_.getX(), pred_centroid_in_ardrone_.getY(), pred_centroid_in_ardrone_.getZ());

    if (current_status_ == Status_n::RELOCALIZING) {
        ROS_INFO("Platform found again!!!");
        ROS_INFO("Tracking...");
        // if we were relocalizing the landing platform, stop the reloc maneuver
        setCmdVelToZero();
        // and set the status to TRACKING
        current_status_ = Status_n::TRACKING;
    }

    // regardless of the state (TAKINGOFF, TRACKING or LANDING), follow the damn platform
    follow_platform(true);
}

void PlatformTracking::centroidPositionCallback(const geometry_msgs::PoseStamped & current_centroid) {
    centroid_in_ardrone_ = tf::Vector3(current_centroid.pose.position.x,
                                       current_centroid.pose.position.y,
                                       current_centroid.pose.position.z);
    centroid_in_ardrone_stamp_ = current_centroid.header.stamp.toSec();

    if (current_status_ == Status_n::RELOCALIZING) {
        ROS_INFO("Platform found again!!!");
        ROS_INFO("Tracking...");
        // if we were relocalizing the landing platform, stop the reloc maneuver
        setCmdVelToZero();
        // and set the status to TRACKING
        current_status_ = Status_n::TRACKING;
    }

    // regardless of the state (TAKINGOFF, TRACKING or LANDING), follow the damn platform
    follow_platform(false);
}

void PlatformTracking::follow_platform(bool from_predicted) {

    if (use_prediction_ && !from_predicted) {
        return;
    }

    if (current_status_ != Status_n::LANDED && current_status_ != Status_n::FORCED_LANDING) {

        /// our target is different depending on whether we use prediction or not
        if (use_prediction_) {
            target_ = pred_centroid_in_ardrone_;
            ROS_INFO("Using prediction to track!");
            ROS_INFO("%f - %f", pred_path_in_world_stamp_, pred_centroid_in_ardrone_.getX());
            ROS_INFO("%f - %f", centroid_in_ardrone_stamp_, centroid_in_ardrone_.getX());
            ROS_INFO("---");

        } else {
            ROS_INFO("NOT using prediction to track!");
            target_ = centroid_in_ardrone_;
        }

        // deal with the time increments
        double current_time = centroid_in_ardrone_stamp_;

        // initializing the system and exiting the function the first time
        if (!seen_centroid_lately_) {
            last_found_time_ = current_time;
            seen_centroid_lately_ = true;
            target_prev_ = target_;
            return;
        }

        /// delta t (time increment)
        dt_ =  current_time - last_found_time_;

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
            // if far away from target, P (proportional action) works just fine (empirically, even better than PID)
            cmd_vel_.linear.x = (Kp_ * target_.getX()) + (Ki_ * error_sum_x_) + (Kd_ * derror_x_);
            cmd_vel_.linear.y = (Kp_ * target_.getY()) + (Ki_ * error_sum_y_) + (Kd_ * derror_y_);
        }

        // store target and current time for next iteration
        target_prev_ = target_;
        last_found_time_ = current_time;

        /// save errors to file
        saveErrorsToCsv(current_time,
                        centroid_in_ardrone_.getX(),
                        centroid_in_ardrone_.getY(),
                        centroid_in_ardrone_.getZ());


        /// TODO: does this bring anything to the table???
        /// WE CALCULTE THE DISTANCE FROM THE QUADROTOR TO THE PLATFORM's axes
        // computeDistanceToPlatformAxes();
        // if (valid_dist_to_axis_) {
        //     if (distance_to_x_axis_platform_ <= 0.01
        //             && !std::isnan(distance_to_x_axis_platform_)
        //             && (dist_in_xy_plane_to_centroid_ > 0.5)) {
        //         setErrorSumToZero("x");
        //     }
        //     if (distance_to_y_axis_platform_ <= 0.01
        //             && !std::isnan(distance_to_y_axis_platform_)
        //             && (dist_in_xy_plane_to_centroid_ > 0.5)) {
        //         setErrorSumToZero("y");
        //     }
        // }
    }
} // end of tracking callback

int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_tracking");
    PlatformTracking platform_tracking;
    ROS_INFO("Set up completed. Ready to start mission");
    ros::spin();
}

