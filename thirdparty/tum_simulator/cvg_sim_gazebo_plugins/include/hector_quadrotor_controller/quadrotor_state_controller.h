/*
* quadrotor_state_controller:
*
* This software is a state control gazebo plugin for the Ardrone simulator
*
* It receives the joystick command and the current simulator information to generate corresponding information in rostopic /ardrone/navdata
*
* Created on: Oct 22, 2012
* Author: Hongrong huang
*
*
*/

#ifndef HECTOR_GAZEBO_PLUGINS_quadrotor_state_controller_H
#define HECTOR_GAZEBO_PLUGINS_quadrotor_state_controller_H

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>

// for camera control
#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>
#include <sensor_msgs/CameraInfo.h>

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


namespace gazebo
{

class GazeboQuadrotorStateController : public ModelPlugin
{
public:
  GazeboQuadrotorStateController();
  virtual ~GazeboQuadrotorStateController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber sonar_subscriber_;
  ros::Subscriber state_subscriber_;

  // extra robot control command
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Subscriber reset_subscriber_;

  ros::Publisher m_navdataPub;

  // extra camera control command
  int                              m_selected_cam_num;
  ros::ServiceServer               toggleCam_service;
  ros::Subscriber                  camera_info_front_subscriber_;
  ros::Subscriber                  camera_info_bottom_subscriber_;
  ros::Publisher                   camera_info_publisher_;
  image_transport::ImageTransport* camera_it_;
  image_transport::Subscriber      camera_front_subscriber_;
  image_transport::Subscriber      camera_bottom_subscriber_;
  image_transport::Publisher       camera_publisher_;

  //***********************************
  
  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  // callback functions for subscribers
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void TakeoffCallback(const std_msgs::EmptyConstPtr&);
  void LandCallback(const std_msgs::EmptyConstPtr&);
  void ResetCallback(const std_msgs::EmptyConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void SonarCallback(const sensor_msgs::RangeConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);
  void CameraFrontCallback(const sensor_msgs::ImageConstPtr&);
  void CameraBottomCallback(const sensor_msgs::ImageConstPtr&);
  void CameraInfoFrontCallback(const sensor_msgs::CameraInfoConstPtr&);
  void CameraInfoBottomCallback(const sensor_msgs::CameraInfoConstPtr&);

  // service functions
  bool setCamChannelCallback(ardrone_autonomy::CamSelect::Request& request, ardrone_autonomy::CamSelect::Response& response);
  bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool toggleNavdataDemoCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool setLedAnimationCallback(ardrone_autonomy::LedAnim::Request& request, ardrone_autonomy::LedAnim::Response& response);

  ros::Time state_stamp;
  math::Pose pose;
  math::Vector3 euler, velocity, acceleration, angular_velocity;
  double robot_altitude;
  // 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
  // 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
  // Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
  unsigned int robot_current_state;

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
  std::string navdata_topic_;

  std::string imu_topic_;
  std::string sonar_topic_;
  std::string state_topic_;

  // extra parameters for robot control.
  bool m_isFlying;
  bool m_takeoff;
  bool m_drainBattery;
  double m_batteryPercentage;
  double m_maxFlightTime;
  double m_timeAfterTakeOff;
  //***********************************

  /// \brief save last_time
  common::Time last_time;
  /// last navigation data
  //ardrone_autonomy::Navdata last_navdata;
  /// filter rate
  //double filter_rate;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // HECTOR_GAZEBO_PLUGINS_quadrotor_state_controller_H
