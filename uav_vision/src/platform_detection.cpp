#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
//#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <cvg_sim_msgs/Altimeter.h>

#include <tf/transform_listener.h>

#include <takeoff/GroundtruthAltitude.h>

namespace enc = sensor_msgs::image_encodings;

#define THRESHOLD_SONAR_ALTIMETER 1e-2
#define PI 3.14159265359


class PlatformDetection {
private:

    bool use_trackbars;

    int H_min_red_, H_max_red_, S_min_red_, S_max_red_, V_min_red_, V_max_red_;
    int H_min_green_, H_max_green_, S_min_green_, S_max_green_, V_min_green_, V_max_green_;

    bool is_cam_info_set_;

    cv::Mat image_, image_display_;

    // to get time stamp of frame
    ros::Time stamp_;
    uint32_t seq_;

    // Topics
    std::string image_topic_, camera_info_topic_;
    std::string platform_position_in_ardrone_topic_, platform_position_in_world_topic_;
    std::string indicator_position_in_ardrone_topic_;
    std::string sonar_height_topic_, altitude_altimeter_topic_;
    std::string gt_altitude_topic_;
    std::string imu_topic_;

    // ROS PUBLISHERS
    ros::Publisher platform_position_in_ardrone_pub_, platform_position_in_world_pub_;
    ros::Publisher indicator_position_in_ardrone_pub_;

    // ROS SUBSCRIBERS
    ros::Subscriber sonar_height_sub_, altitude_altimeter_sub_;
    ros::Subscriber gt_altitude_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber imu_sub_;

    // image Subscriber and Publisher
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // cam info data
    double fx_, fy_, cx_, cy_, T_;

    // altitude data
    double sonar_range_, altitude_altimeter_;
    double quadrotor_groundtruth_z_;
    double z_dist_to_platform_;

    // IMU data
    double angle_x_, angle_y_;
    double angular_velocity_x_, angular_velocity_y_;
    double linear_acceleration_x_, linear_acceleration_y_;

    // dimensions of landing platform
    double height_landing_platform_;

    // VARIABLES FOR DRAWING THE LANDING_PLATFORM SHAPE
    std::vector< std::vector<cv::Point> > contours_;
    std::vector<cv::Vec4i> hierarchy_;
    std::vector<cv::Point> approx_;

    // transformations
    tf::TransformListener tf_listener_;
    tf::StampedTransform T_ardrone_cam_, T_world_cam_;

    // 3d positions
    tf::Vector3 position_in_camera_, position_in_ardrone_, position_in_world_;

    void computeVerticalDistToPlatform();

public:
    PlatformDetection();
    void camInfoCallback(const sensor_msgs::CameraInfo& cam_info_msg);
    void inputImageCallback(const sensor_msgs::ImageConstPtr& raw_image);

    void sonarCallback(const sensor_msgs::RangeConstPtr& sonar_msg);
    void altimeterCallback(const cvg_sim_msgs::AltimeterConstPtr& altimeter_msg);
    void groundtruthAltitudeCallback(const takeoff::GroundtruthAltitudeConstPtr& gt_altitude_msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

    void computeCentroidOrIndicator(const std::string& type);

}; //end of class platform_Detection


PlatformDetection::PlatformDetection() {
    ROS_INFO("Init Class: Platform_Detection");

    ros::NodeHandle nh_("~");
    image_transport::ImageTransport it_(nh_);

    is_cam_info_set_ = false;
    use_trackbars = false;

    // GETTING ROS PARAMETERS
    nh_.getParam("use_trackbars", use_trackbars);
    nh_.getParam("image_topic", image_topic_);
    nh_.getParam("cam_info_topic", camera_info_topic_);
    nh_.getParam("platform_position_in_ardrone_topic", platform_position_in_ardrone_topic_);
    nh_.getParam("platform_position_in_world_topic", platform_position_in_world_topic_);
    nh_.getParam("indicator_position_in_ardrone_topic", indicator_position_in_ardrone_topic_);
    nh_.getParam("sonar_topic", sonar_height_topic_);
    nh_.getParam("altimeter_topic", altitude_altimeter_topic_);
    nh_.getParam("gt_altitude_topic", gt_altitude_topic_);
    nh_.getParam("imu_topic", imu_topic_);

    nh_.getParam("height_landing_platform", height_landing_platform_);

    // --TOPICS -- //
    image_sub_              = it_.subscribe(image_topic_, 1, &PlatformDetection::inputImageCallback, this);
    image_pub_              = it_.advertise("/image_converter/output_video", 1);

    cam_info_sub_           = nh_.subscribe(camera_info_topic_, 1, &PlatformDetection::camInfoCallback, this);

    sonar_height_sub_       = nh_.subscribe(sonar_height_topic_, 1, &PlatformDetection::sonarCallback, this);
    altitude_altimeter_sub_ = nh_.subscribe(altitude_altimeter_topic_, 1, &PlatformDetection::altimeterCallback, this);
    gt_altitude_sub_        = nh_.subscribe(gt_altitude_topic_, 1,
                                            &PlatformDetection::groundtruthAltitudeCallback, this);
    imu_sub_                = nh_.subscribe(imu_topic_, 1, &PlatformDetection::imuCallback, this);

    platform_position_in_ardrone_pub_  = nh_.advertise<geometry_msgs::PoseStamped>(platform_position_in_ardrone_topic_, 1);
    platform_position_in_world_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(platform_position_in_world_topic_, 1);
    indicator_position_in_ardrone_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(indicator_position_in_ardrone_topic_, 1);

    // --TRACKBARS-- //
    H_min_red_ = 0;   H_max_red_ = 10;
    S_min_red_ = 100; S_max_red_ = 255;
    V_min_red_ = 100; V_max_red_ = 255;

    H_min_green_ = 44;   H_max_green_ = 71;
    S_min_green_ = 100; S_max_green_ = 255;
    V_min_green_ = 100; V_max_green_ = 255;

    if (use_trackbars) {
        cv::namedWindow("Platform");
        cv::createTrackbar("H_min", "Platform", &H_min_red_, 179, NULL);
        cv::createTrackbar("H_max", "Platform", &H_max_red_, 179, NULL);
        cv::createTrackbar("S_min", "Platform", &S_min_red_, 255, NULL);
        cv::createTrackbar("S_max", "Platform", &S_max_red_, 255, NULL);
        cv::createTrackbar("V_min", "Platform", &V_min_red_, 255, NULL);
        cv::createTrackbar("V_max", "Platform", &V_max_red_, 255, NULL);

        cv::namedWindow("Indicator");
        cv::createTrackbar("H_min", "Indicator", &H_min_green_, 179, NULL);
        cv::createTrackbar("H_max", "Indicator", &H_max_green_, 179, NULL);
        cv::createTrackbar("S_min", "Indicator", &S_min_green_, 255, NULL);
        cv::createTrackbar("S_max", "Indicator", &S_max_green_, 255, NULL);
        cv::createTrackbar("V_min", "Indicator", &V_min_green_, 255, NULL);
        cv::createTrackbar("V_max", "Indicator", &V_max_green_, 255, NULL);
    }

    try {
        ros::Time now = ros::Time::now();
        tf_listener_.waitForTransform("/ardrone/base_link", "/ardrone/ardrone_base_bottomcam",
                                      now, ros::Duration(3.0));
        tf_listener_.lookupTransform("/ardrone/base_link", "/ardrone/ardrone_base_bottomcam",
                                     now, T_ardrone_cam_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
        ros::Time now = ros::Time::now();
        tf_listener_.waitForTransform("/odom", "/ardrone/ardrone_base_bottomcam",
                                      now, ros::Duration(3.0));
        tf_listener_.lookupTransform("/odom", "/ardrone/ardrone_base_bottomcam",
                                     now, T_world_cam_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Setup completed");
}

void PlatformDetection::sonarCallback(const sensor_msgs::RangeConstPtr& sonar_msg) {
    sonar_range_ = sonar_msg->range;
    // ROS_INFO("sonar: %f\n", sonar_range_);
}

void PlatformDetection::altimeterCallback(const cvg_sim_msgs::AltimeterConstPtr& altimeter_msg) {
    altitude_altimeter_ = altimeter_msg->altitude * 100;
    // ROS_INFO("altimeter: %f\n", altitude_altimeter_);
}

void PlatformDetection::groundtruthAltitudeCallback(const takeoff::GroundtruthAltitudeConstPtr& gt_altitude_msg) {
    quadrotor_groundtruth_z_ = gt_altitude_msg->gt_altitude;;
    // ROS_INFO("gt: %f\n", quadrotor_groundtruth_z_);
}

void PlatformDetection::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    angle_x_               = imu_msg->orientation.x;
    angular_velocity_x_    = imu_msg->angular_velocity.x;

    angle_y_               = imu_msg->orientation.y;
    angular_velocity_y_    = imu_msg->angular_velocity.y;

    linear_acceleration_x_ = imu_msg->linear_acceleration.x;
    linear_acceleration_y_ = imu_msg->linear_acceleration.y;
}

void PlatformDetection::camInfoCallback(const sensor_msgs::CameraInfo& cam_info_msg) {

    if (!is_cam_info_set_) {
        fx_ = cam_info_msg.K.at(0);
        fy_ = cam_info_msg.K.at(4);
        cx_ = cam_info_msg.K.at(2);
        cy_ = cam_info_msg.K.at(5);
        T_  = cam_info_msg.K.at(1);
        ROS_INFO("Parameters from the pinhole model");
        ROS_INFO("(fx,fy) = (%f , %f)", fx_, fy_);
        ROS_INFO("(cx,cy) = (%f , %f)", cx_, cy_);
        ROS_INFO("T = %f", T_);
        is_cam_info_set_ = true;
    }
}

void PlatformDetection::computeVerticalDistToPlatform() {
    double abs_diff = abs(sonar_range_ - altitude_altimeter_);
    if (abs_diff < THRESHOLD_SONAR_ALTIMETER) {
        // ardrone is actually hovering directly above landing platform
        z_dist_to_platform_ = altitude_altimeter_;
    } else if ( (abs_diff - height_landing_platform_) < THRESHOLD_SONAR_ALTIMETER) {
        // ardrone is NOT directly above landing platform
        z_dist_to_platform_ = altitude_altimeter_ - height_landing_platform_;
    }
    // ROS_INFO("z-distance to platform: %.2f", z_dist_to_platform_);
}

void PlatformDetection::inputImageCallback(const sensor_msgs::ImageConstPtr& raw_image) {

    // discard frame if angle_x_ or angle_y_ bigger than THRESHOLD
    // ROS_INFO_STREAM("angle_x " << angle_x_ << " - angle_y " << angle_y_);
    // ROS_INFO_STREAM(PI / 36);

    if ( fabs(angle_x_) >= PI / 45.0 || fabs(angle_y_) >= PI / 45.0)
        return;

    /*if (sonar_range_ < 0.05)
        return;*/

    // get time stamp for frame
    stamp_ = ros::Time::now();
    seq_ = stamp_.toSec();

    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(raw_image, enc::BGR8);
        image_ = cv_ptr->image;
        image_display_ = image_.clone();
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Failed to convert image: %s", e.what());
        return;
    }

    computeCentroidOrIndicator("centroid");
    computeCentroidOrIndicator("indicator");


    cv::imshow("platform_DETECTED", image_display_);
    cv::waitKey(1);
}

void PlatformDetection::computeCentroidOrIndicator(const std::string& type) {
    cv::Scalar color; // for drawing the detected platform edges

    cv::Mat img_hsv;
    cv::cvtColor(image_, img_hsv, CV_BGR2HSV);

    cv::Mat img_masked;
    if (type == "centroid") {
        color = cv::Scalar(0, 255, 0);
        cv::inRange(img_hsv, cv::Scalar(H_min_red_, S_min_red_, S_min_red_),
                    cv::Scalar(H_max_red_, S_max_red_, V_max_red_), img_masked);
        // cv::imshow("hsv_masked", img_masked);
    }
    else if (type == "indicator") {
        color = cv::Scalar(255, 0, 0);
        cv::inRange(img_hsv, cv::Scalar(H_min_green_, S_min_green_, V_min_green_),
                    cv::Scalar(H_max_green_, S_max_green_, V_max_green_), img_masked);
        // cv::imshow("hsv_masked ind", img_masked);
    }


    // cv::Mat img_negated;
    // cv::bitwise_not(img_masked, img_negated);
    // cv::imshow("bitwise_not", img_negated);

    // cv::Mat img_blured;
    // cv::medianBlur(img_negated, img_blured, 5);
    // cv::imshow("blur", img_blured);

    //Find countours on the image
    contours_.clear();
    hierarchy_.clear();
    approx_.clear();
    cv::findContours(img_masked, contours_, hierarchy_, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours_.size(); i++) {
        cv::approxPolyDP(contours_[i], approx_, cv::arcLength(contours_[i], true) * 0.02, true);

        cv::Rect bounding_box = cv::boundingRect(contours_[i]);
        const int area_box = bounding_box.area();

        bool is_whole_image = (quadrotor_groundtruth_z_ > 1.0) && (area_box > (image_.cols * image_.rows - 400));
        if (area_box < 50 || is_whole_image)
            continue;

        cv::Point pt;
        for (int j = 0; j < approx_.size(); j++) {
            if (j == approx_.size() - 1)
                cv::line(image_display_, approx_.at(j), approx_.at(0), color, 2);
            else
                cv::line(image_display_, approx_.at(j), approx_.at(j + 1), color, 2);
        }

        /// ########## Centroid extraction ##########
        /// 1º Get the moments
        cv::Moments m;
        m = cv::moments( approx_, false );

        /// 2º Get the mass centers: (centroid)
        cv::Point2f mc;
        mc = cv::Point2f( m.m10 / m.m00 , m.m01 / m.m00 );
        cv::line(image_display_, mc, mc, cv::Scalar(0, 255, 255), 5);

        // ---------------------------------------------------------------------------------------

        /// Now we have to compute the 3d coordinates of the found centroid --
        /// generate position of centroid in the camera frame
        /// PINHOLE inverse transformation ==> (x,y,z)= f(u,v) without distortion, (u,v) = mc
        z_dist_to_platform_ = quadrotor_groundtruth_z_ - height_landing_platform_;
        double centroid_x_in_cam = ((mc.x - cx_) * z_dist_to_platform_ - (T_ * (mc.y - cy_) * z_dist_to_platform_) / fy_) / fx_;
        double centroid_y_in_cam = ((mc.y - cy_) * z_dist_to_platform_) / fy_;
        double centroid_z_in_cam = z_dist_to_platform_;

        if (std::isnan(centroid_x_in_cam) || std::isnan(centroid_y_in_cam) || std::isnan(centroid_z_in_cam)) {
            return;
        }

        position_in_camera_ = tf::Vector3(centroid_x_in_cam, centroid_y_in_cam, centroid_z_in_cam);

        /// convert to the ardrone's base_link frame
        try {
            tf_listener_.waitForTransform("/ardrone/base_link", "/ardrone/ardrone_base_bottomcam",
                                          stamp_, ros::Duration(3.0));
            tf_listener_.lookupTransform("/ardrone/base_link", "/ardrone/ardrone_base_bottomcam",
                                         stamp_, T_ardrone_cam_);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // position in base_link
        position_in_ardrone_ = T_ardrone_cam_ * position_in_camera_;

        geometry_msgs::PoseStamped pose_in_ardrone;
        pose_in_ardrone.header.stamp = stamp_;
        pose_in_ardrone.header.seq = seq_;
        pose_in_ardrone.header.frame_id = "/ardrone/base_link";
        pose_in_ardrone.pose.orientation.w = 1;
        pose_in_ardrone.pose.orientation.x = 0;
        pose_in_ardrone.pose.orientation.y = 0;
        pose_in_ardrone.pose.orientation.z = 0;
        pose_in_ardrone.pose.position.x = position_in_ardrone_.getX();
        pose_in_ardrone.pose.position.y = position_in_ardrone_.getY();
        pose_in_ardrone.pose.position.z = position_in_ardrone_.getZ();


        if (type == "centroid") {

            platform_position_in_ardrone_pub_.publish(pose_in_ardrone);

            // ---------------------------------------------------------------------------------------

            // convert to the world's frame
            try {
                tf_listener_.waitForTransform("/odom", "/ardrone/ardrone_base_bottomcam",
                                              stamp_, ros::Duration(3.0));
                tf_listener_.lookupTransform("/odom", "/ardrone/ardrone_base_bottomcam",
                                             stamp_, T_world_cam_);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            // position in world (more precisely, odom)
            position_in_world_ = T_world_cam_ * position_in_camera_;

            geometry_msgs::PoseStamped pose_in_world;
            pose_in_world.header.stamp = stamp_;
            pose_in_world.header.seq = seq_;
            pose_in_world.header.frame_id = "/odom";
            pose_in_world.pose.orientation.w = 1;
            pose_in_world.pose.orientation.x = 0;
            pose_in_world.pose.orientation.y = 0;
            pose_in_world.pose.orientation.z = 0;
            pose_in_world.pose.position.x = position_in_world_.getX();
            pose_in_world.pose.position.y = position_in_world_.getY();
            pose_in_world.pose.position.z = position_in_world_.getZ();

            platform_position_in_world_pub_.publish(pose_in_world);

            // ROS_INFO("Position in world (%f, %f, %f)", position_in_world_.getX(), position_in_world_.getY(), position_in_world_.getZ());
        }
        else if (type == "indicator") {
            indicator_position_in_ardrone_pub_.publish(pose_in_ardrone);
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_detection");
    PlatformDetection platform;
    ROS_INFO("platform_detection::main.cpp - Spinning");
    ros::spin();
    ROS_INFO("platform_detection::main.cpp - Exited correctly");
    cv::destroyAllWindows();

    return 0;
}