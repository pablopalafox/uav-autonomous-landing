#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

namespace RobotLocalization
{
  class NavSatTransformNew
  {
  public:
     NavSatTransformNew();

     ~NavSatTransformNew();     
     void run();
     
  protected:
     
     void odomCallback(const nav_msgs::OdometryConstPtr& msg);     
     void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);
     void imuCallback(const sensor_msgs::ImuConstPtr& msg);
     
     void localOdomCallback(const nav_msgs::OdometryConstPtr& msg);

     void computeTransform();
     bool prepareGpsOdometry(nav_msgs::Odometry &gpsOdom);

     void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg);
     void setTransformOdometry(const nav_msgs::OdometryConstPtr& msg);

     //MEMBERS FROM NAVSAT_TRANSFORM OF ROBOT_LOCALIZATION
     
     bool broadcastUtmTransform_;
     
     double magneticDeclination_;
     double utmOdomTfYaw_;

     bool hasTransformGps_;
     bool hasTransformOdom_;
     bool hasTransformImu_;
     bool transformGood_;

     bool gpsUpdated_;
     bool odomUpdated_;

     ros::Time gpsUpdateTime_;

     ros::Time odomUpdateTime_;

     double yawOffset_;

     bool zeroAltitude_;

     bool publishGps_;

     std::string baseLinkFrameId_;
     std::string worldFrameId_;

     std::string utmZone_;
     
     tf2::Transform latestWorldPose_;
     tf2::Transform latestUtmPose_;

     tf2::Transform transformUtmPose_;
     tf2::Transform transformWorldPose_;
     
     tf2::Quaternion transformOrientation_;

     Eigen::MatrixXd latestUtmCovariance_;  
     Eigen::MatrixXd latestOdomCovariance_;
  
     tf2_ros::Buffer tfBuffer_;
     tf2_ros::TransformListener tfListener_;
     tf2_ros::StaticTransformBroadcaster utmBroadcaster_;

     tf2::Transform utmWorldTransform_;
     tf2::Transform utmWorldTransInverse_;

     //NEW MEMBERS
     
     //! @brief Parameter that specifies the travelledDistance after which update the odometry provided by GPS
     //! if negative, the update is continuous (standard NavSatTransform node behaviour)
     double travelledDistance_;
     double previous_x_;
     double previous_y_;
      
     //! @brief Parameter that specifies the topic from which get the travelled distance to update GPS odometry
     std::string localOdomTopic_;
     bool hasToPublish_;
  };
}
