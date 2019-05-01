/*
 * summit_xl_robot_control_vrep
 * Copyright (c) 2014, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik
 * \brief Controller for the Summit XL robot in skid-steering / mecanum omni-drive modes.
 * \brief simulation of 4DOF Mecanum kinematics by equivalent 8DOF Swerve drive
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/get_mode.h>
#include <robotnik_msgs/set_odometry.h>
#include <robotnik_msgs/ptz.h>


//#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"


#define PI 3.1415926535
#define SUMMIT_XL_MIN_COMMAND_REC_FREQ   5.0
#define SUMMIT_XL_MAX_COMMAND_REC_FREQ   150.0

#define SKID_STEERING                1
#define MECANUM_STEERING             2

#define SUMMIT_XL_WHEEL_DIAMETER	0.25      // Default wheel diameter
#define SUMMIT_XL_D_TRACKS_M    	1.0       // default equivalent W distance (difference is due to slippage of skid steering)
#define SUMMIT_XL_WHEELBASE         0.446     // default real L distance forward to rear axis
#define SUMMIT_XL_TRACKWIDTH        0.408     // default real W distance from left wheels to right wheels
    
using namespace std;

class SummitXLControllerClass {

public:

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;

  // Diagnostics
  diagnostic_updater::Updater diagnostic_;			// General status diagnostic updater
  diagnostic_updater::FrequencyStatus freq_diag_;		         // Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
  ros::Time last_command_time_;					// Last moment when the component received a command
  diagnostic_updater::FunctionDiagnosticTask command_freq_;

  // Robot model 
  std::string robot_model_;
  
  // Velocity and position references to low level controllers
  ros::Publisher ref_vel_flw_;
  ros::Publisher ref_vel_frw_;
  ros::Publisher ref_vel_blw_;
  ros::Publisher ref_vel_brw_;
  ros::Publisher ref_pos_flw_;
  ros::Publisher ref_pos_frw_;
  ros::Publisher ref_pos_blw_;
  ros::Publisher ref_pos_brw_; 
  ros::Publisher ref_pos_scissor_;
  ros::Publisher ref_pos_pan_;
  ros::Publisher ref_pos_tilt_;

  // Joint states published by the joint_state_controller of the Controller Manager
  ros::Subscriber joint_state_sub_;

  // High level robot command
  ros::Subscriber cmd_sub_;
	
  // High level robot command
  ros::Subscriber ptz_sub_;

  //ros::Subscriber gyro_sub_;

  // Services
  ros::ServiceServer srv_SetOdometry_;
  ros::ServiceServer srv_SetMode_;
  ros::ServiceServer srv_GetMode_;
  
  // Topics - skid - velocity
  std::string frw_vel_topic_;
  std::string flw_vel_topic_;
  std::string brw_vel_topic_;
  std::string blw_vel_topic_;
  
  // Joint names - skid - velocity 
  std::string joint_front_right_wheel;
  std::string joint_front_left_wheel;
  std::string joint_back_left_wheel;
  std::string joint_back_right_wheel;

  // Topics - swerve - position
  std::string frw_pos_topic_;
  std::string flw_pos_topic_;
  std::string brw_pos_topic_;
  std::string blw_pos_topic_;
    
  // Joint names - swerve - position
  std::string joint_front_right_steer;
  std::string joint_front_left_steer;
  std::string joint_back_left_steer;
  std::string joint_back_right_steer;

  // Topic - scissor - position
  std::string scissor_pos_topic_;

  // Joint names - ptz - position
  std::string joint_camera_pan;
  std::string joint_camera_tilt;

  // Topics - ptz
  std::string pan_pos_topic_;
  std::string tilt_pos_topic_;
    
  // Joint name - scissor mechanism
  std::string scissor_prismatic_joint;

  // Selected operation mode
  int kinematic_modes_;   
  int active_kinematic_mode_;

  // Indexes to joint_states
  int frw_vel_, flw_vel_, blw_vel_, brw_vel_;
  int frw_pos_, flw_pos_, blw_pos_, brw_pos_;
  int scissor_pos_;
  int pan_pos_, tilt_pos_;


  // Robot Speeds
  double linearSpeedXMps_;
  double linearSpeedYMps_;
  double angularSpeedRads_;

  // Robot Positions
  double robot_pose_px_;
  double robot_pose_py_;
  double robot_pose_pa_;
  double robot_pose_vx_;
  double robot_pose_vy_;
  
  // Robot Joint States
  sensor_msgs::JointState joint_state_;
  
  // Command reference
  geometry_msgs::Twist base_vel_msg_;

  // External speed references
  double v_ref_x_;
  double v_ref_y_;
  double w_ref_;
  double v_ref_z_;
  double pos_ref_pan_;
  double pos_ref_tilt_;
  
  // Flag to indicate if joint_state has been read
  bool read_state_; 
  
  // Robot configuration parameters 
  double summit_xl_wheel_diameter_; 
  double summit_xl_d_tracks_m_;
  double summit_xl_wheelbase_;
  double summit_xl_trackwidth_;

  // IMU values
  double ang_vel_x_;
  double ang_vel_y_;
  double ang_vel_z_;

  double lin_acc_x_;
  double lin_acc_y_;
  double lin_acc_z_;

  double orientation_x_;
  double orientation_y_;
  double orientation_z_;
  double orientation_w_;

  // Parameter that defines if odom tf is published or not
  bool publish_odom_tf_;

  ros::Subscriber imu_sub_; 
  
  // Publisher for odom topic
  ros::Publisher odom_pub_; 

  // Broadcaster for odom tf  
  tf::TransformBroadcaster odom_broadcaster;


/*!	\fn SummitXLControllerClass::SummitXLControllerClass()
 * 	\brief Public constructor
*/
SummitXLControllerClass(ros::NodeHandle h) : diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  desired_freq_(100),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&SummitXLControllerClass::check_command_subscriber, this, _1))
  {

  // /summit_xl/joint_blw_velocity_controller/joint
  ROS_INFO("summit_xl_robot_control_node - Init ");

  // 4-Axis Skid Steer Rover
  // 8-Axis Omni-drive as 4-Axis Mecanum drive
  kinematic_modes_ = 1;  
  
  ros::NodeHandle summit_xl_robot_control_node_handle(node_handle_, "summit_xl_robot_control");

  // Get robot model from the parameters
  if (!private_node_handle_.getParam("model", robot_model_)) {
	  ROS_ERROR("Robot model not defined.");
	  exit(-1);
	  }
  else ROS_INFO("Robot Model : %s", robot_model_.c_str());

  // Skid configuration - topics
  private_node_handle_.param<std::string>("frw_vel_topic", frw_vel_topic_, "/summit_xl/joint_frw_velocity_controller/command");
  private_node_handle_.param<std::string>("flw_vel_topic", flw_vel_topic_, "/summit_xl/joint_flw_velocity_controller/command");
  private_node_handle_.param<std::string>("blw_vel_topic", blw_vel_topic_, "/summit_xl/joint_blw_velocity_controller/command");
  private_node_handle_.param<std::string>("brw_vel_topic", brw_vel_topic_, "/summit_xl/joint_brw_velocity_controller/command");

  // Skid configuration - Joint names 
  private_node_handle_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "joint_front_right_wheel");
  private_node_handle_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "joint_front_left_wheel");
  private_node_handle_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "joint_back_left_wheel");
  private_node_handle_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "joint_back_right_wheel");

  // Omni configuration - topics
  if ((robot_model_=="summit_xl_omni") || (robot_model_=="x_wam")) {
	kinematic_modes_ = 2;
    private_node_handle_.param<std::string>("frw_pos_topic", frw_pos_topic_, "/summit_xl/joint_frw_velocity_controller/command");
    private_node_handle_.param<std::string>("flw_pos_topic", flw_pos_topic_, "/summit_xl/joint_flw_velocity_controller/command");
    private_node_handle_.param<std::string>("blw_pos_topic", blw_pos_topic_, "/summit_xl/joint_blw_velocity_controller/command");
    private_node_handle_.param<std::string>("brw_pos_topic", brw_pos_topic_, "/summit_xl/joint_brw_velocity_controller/command");

    private_node_handle_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "joint_front_right_steer");
    private_node_handle_.param<std::string>("joint_front_left_steer", joint_front_left_steer, "joint_front_left_steer");
    private_node_handle_.param<std::string>("joint_back_left_steer", joint_back_left_steer, "joint_back_left_steer");
    private_node_handle_.param<std::string>("joint_back_right_steer", joint_back_right_steer, "joint_back_right_steer");

    if (!private_node_handle_.getParam("summit_xl_wheelbase", summit_xl_wheelbase_))
        summit_xl_wheelbase_ = SUMMIT_XL_WHEELBASE;
    if (!private_node_handle_.getParam("summit_xl_trackwidth", summit_xl_trackwidth_))
        summit_xl_trackwidth_ = SUMMIT_XL_TRACKWIDTH;

    // x-wam only configuration
    if (robot_model_=="x_wam") {
      private_node_handle_.param<std::string>("scissor_pos_topic", scissor_pos_topic_, "/summit_xl/joint_scissor_position_controller/command");		
	  private_node_handle_.param<std::string>("scissor_prismatic_joint", scissor_prismatic_joint, "scissor_prismatic_joint");
      } 
    }

  // PTZ topics
  private_node_handle_.param<std::string>("pan_pos_topic", pan_pos_topic_, "/summit_xl/joint_pan_position_controller/command");
  private_node_handle_.param<std::string>("tilt_pos_topic", tilt_pos_topic_, "/summit_xl/joint_tilt_position_controller/command");
  private_node_handle_.param<std::string>("joint_camera_pan", joint_camera_pan, "joint_camera_pan");
  private_node_handle_.param<std::string>("joint_camera_tilt", joint_camera_tilt, "joint_camera_tilt");

  // Robot parameters
  if (!private_node_handle_.getParam("summit_xl_wheel_diameter", summit_xl_wheel_diameter_))
    summit_xl_wheel_diameter_ = SUMMIT_XL_WHEEL_DIAMETER;
  if (!private_node_handle_.getParam("summit_xl_d_tracks_m", summit_xl_d_tracks_m_))
    summit_xl_d_tracks_m_ = SUMMIT_XL_D_TRACKS_M;
  ROS_INFO("summit_xl_wheel_diameter_ = %5.2f", summit_xl_wheel_diameter_);
  ROS_INFO("summit_xl_d_tracks_m_ = %5.2f", summit_xl_d_tracks_m_);

  private_node_handle_.param("publish_odom_tf", publish_odom_tf_, true);
  if (publish_odom_tf_) ROS_INFO("PUBLISHING odom->base_footprin tf");
  else ROS_INFO("NOT PUBLISHING odom->base_footprint tf");
  
  // Robot Speeds
  linearSpeedXMps_   = 0.0;
  linearSpeedYMps_   = 0.0;
  angularSpeedRads_  = 0.0;

  // Robot Positions
  robot_pose_px_ = 0.0;
  robot_pose_py_ = 0.0;
  robot_pose_pa_ = 0.0;
  robot_pose_vx_ = 0.0;
  robot_pose_vy_ = 0.0;

  // External speed references
  v_ref_x_ = 0.0;
  v_ref_y_ = 0.0;
  w_ref_ = 0.0;
  v_ref_z_ = 0.0;
  pos_ref_pan_ = 0.0;
  pos_ref_tilt_= 0.0;

  // Imu variables
  ang_vel_x_ = 0.0; ang_vel_y_ = 0.0; ang_vel_z_ = 0.0;
  lin_acc_x_ = 0.0; lin_acc_y_ = 0.0; lin_acc_z_ = 0.0;
  orientation_x_ = 0.0; orientation_y_ = 0.0; orientation_z_ = 0.0; orientation_w_ = 0.0;

  // Active kinematic mode
  active_kinematic_mode_ = SKID_STEERING;

  // Advertise services
  srv_SetMode_ = summit_xl_robot_control_node_handle.advertiseService("set_mode", &SummitXLControllerClass::srvCallback_SetMode, this);
  srv_GetMode_ = summit_xl_robot_control_node_handle.advertiseService("get_mode", &SummitXLControllerClass::srvCallback_GetMode, this);
  srv_SetOdometry_ = summit_xl_robot_control_node_handle.advertiseService("set_odometry",  &SummitXLControllerClass::srvCallback_SetOdometry, this);

  // Subscribe to joint states topic
  joint_state_sub_ = summit_xl_robot_control_node_handle.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &SummitXLControllerClass::jointStateCallback, this);

  // Subscribe to imu data
  imu_sub_ = summit_xl_robot_control_node_handle.subscribe("/summit_xl/imu_data", 1, &SummitXLControllerClass::imuCallback, this);

  // Adevertise reference topics for the controllers 
  ref_vel_frw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( frw_vel_topic_, 50);
  ref_vel_flw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( flw_vel_topic_, 50);
  ref_vel_blw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( blw_vel_topic_, 50);
  ref_vel_brw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( brw_vel_topic_, 50);
  
  if ((robot_model_=="summit_xl_omni")||(robot_model_=="x_wam")) {
	  ref_pos_frw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( frw_pos_topic_, 50);
	  ref_pos_flw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( flw_pos_topic_, 50);
	  ref_pos_blw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( blw_pos_topic_, 50);	  
	  ref_pos_brw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( brw_pos_topic_, 50);
  
	  if (robot_model_=="x_wam")
	     ref_pos_scissor_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( scissor_pos_topic_, 50);	     
	  }
	  
  ref_pos_pan_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( pan_pos_topic_, 50);
  ref_pos_tilt_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( tilt_pos_topic_, 50);

  // Subscribe to command topic
  cmd_sub_ = summit_xl_robot_control_node_handle.subscribe<geometry_msgs::Twist>("command", 1, &SummitXLControllerClass::commandCallback, this);

  // Subscribe to ptz command topic
  ptz_sub_ = summit_xl_robot_control_node_handle.subscribe<robotnik_msgs::ptz>("command_ptz", 1, &SummitXLControllerClass::command_ptzCallback, this);
  // /summit_xl_robot_control/command_ptz
  
  // TODO odom topic as parameter
  // Publish odometry 
  odom_pub_ = summit_xl_robot_control_node_handle.advertise<nav_msgs::Odometry>("/summit_xl/odom", 1000);

  // Component frequency diagnostics
  diagnostic_.setHardwareID("summit_xl_robot_control - simulation");
  diagnostic_.add( freq_diag_ );
  diagnostic_.add( command_freq_ );
    
  // Topics freq control 
  // For /summit_xl_robot_control/command
  double min_freq = SUMMIT_XL_MIN_COMMAND_REC_FREQ; // If you update these values, the
  double max_freq = SUMMIT_XL_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
  subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/summit_xl_robot_control/command", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
  subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control
  
  // Flag to indicate joint_state has been read
  read_state_ = false;
}

/// Controller startup in realtime
int starting()
{

  ROS_INFO("SummitXLControllerClass::starting");

  //name: ['joint_back_left_wheel', 'joint_back_right_wheel', 'joint_front_left_wheel', 'joint_front_right_wheel']
  //position: [-0.04246698357387224, 0.053199274627900195, -0.04246671523622059, 0.03126368464965523]
  //velocity: [-0.0006956167983269147, 0.0004581098383479411, -0.0013794952191663358, 0.00045523862784977847]

  // Initialize joint indexes according to joint names 
  if (read_state_) {
    vector<string> joint_names = joint_state_.name;
    frw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_wheel)) - joint_names.begin();
    flw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_wheel)) - joint_names.begin();
    blw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_wheel)) - joint_names.begin();
    brw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_wheel)) - joint_names.begin();
    if ((robot_model_=="summit_xl_omni")||(robot_model_=="x_wam")) {
      frw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_steer)) - joint_names.begin();
      flw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_steer)) - joint_names.begin();
      blw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_steer)) - joint_names.begin();
      brw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_steer)) - joint_names.begin();
	  }
	if (robot_model_=="x_wam") {
	  scissor_pos_ = find (joint_names.begin(),joint_names.end(), string(scissor_prismatic_joint)) - joint_names.begin();
	  }
	// For publishing the ptz joint state   
	//pan_pos_ = find(joint_names.begin(), joint_names.end(), string(joint_camera_pan)) - joint_names.begin();
	//tilt_pos_ = find(joint_names.begin(), joint_names.end(), string(joint_camera_tilt)) - joint_names.begin();
    return 0;
    }
  else return -1;
}


/// Controller update loop 
void UpdateControl()
{

  // Depending on the robot configuration 
  if (active_kinematic_mode_ == SKID_STEERING) {

	  // Inverse kinematics
	  double dUl = v_ref_x_ - 0.5 * summit_xl_d_tracks_m_ * w_ref_;
      // sign according to urdf (if wheel model is not symetric, should be inverted)
   	  double dUr = v_ref_x_ + 0.5 * summit_xl_d_tracks_m_ * w_ref_;  

	  // Motor control actions
	  double limit = 40.0;   // 40.0;
	  
	  //ROS_INFO("epv=%5.2f, epw=%5.2f ***  dUl=%5.2f  dUr=%5.2f", epv, epw, dUl, dUr);

	  // Axis are not reversed in the omni (swerve) configuration
      std_msgs::Float64 frw_ref_msg; 
      std_msgs::Float64 flw_ref_msg;
      std_msgs::Float64 blw_ref_msg;
      std_msgs::Float64 brw_ref_msg;
	  
      double k1 = 5.0; 
	  frw_ref_msg.data = -saturation( k1 * dUr, -limit, limit);  
	  flw_ref_msg.data = saturation( k1 * dUl, -limit, limit);
      blw_ref_msg.data = saturation( k1 * dUl, -limit, limit);
	  brw_ref_msg.data = -saturation( k1 * dUr, -limit, limit);
	  
	  ref_vel_frw_.publish( frw_ref_msg );
	  ref_vel_flw_.publish( flw_ref_msg );
	  ref_vel_blw_.publish( blw_ref_msg );
	  ref_vel_brw_.publish( brw_ref_msg );
	  }
	
/*	
  // Depending on the robot configuration 
  if (active_kinematic_mode_ == SKID_STEERING) {

  	  double v_left_mps, v_right_mps;

	  // Calculate its own velocities for realize the motor control 
	  v_left_mps = ((joint_state_.velocity[blw_vel_] + joint_state_.velocity[flw_vel_]) / 2.0) * (summit_xl_wheel_diameter_ / 2.0);
	  v_right_mps = -((joint_state_.velocity[brw_vel_] + joint_state_.velocity[frw_vel_]) / 2.0) * (summit_xl_wheel_diameter_ / 2.0); 
	  // sign according to urdf (if wheel model is not symetric, should be inverted)

	  linearSpeedXMps_ = (v_right_mps + v_left_mps) / 2.0;                       // m/s
	  angularSpeedRads_ = (v_right_mps - v_left_mps) / summit_xl_d_tracks_m_;    // rad/s

	  ROS_INFO("vleft=%5.2f   vright=%5.2f    linearSpeedXMps=%5.2f, linearSpeedYMps=%5.2f, angularSpeedRads=%5.4f", v_left_mps, v_right_mps,
			linearSpeedXMps_, linearSpeedYMps_, angularSpeedRads_); 

	  // Current controllers close this loop allowing (v,w) references.
	  double epv=0.0;
	  double epw=0.0;
	  static double epvant =0.0;
	  static double epwant =0.0;

	  // Adjusted for soft indoor office soil
	  //double kpv=2.5; double kdv=0.0;
	  //double kpw=2.5;  double kdw=0.0;
	  // In order to use this controller with the move_base stack note that move_base sets the min rotation ref to 0.4 independantly of the yaml configurations!!!
	  //double kpv=2.5; double kdv=0.0;
	  //double kpw=25.0; double kdw=15.0;
	  
	  // For VREP
	  double kpv=2.5; double kdv=0.0;
	  double kpw=2.5;  double kdw=0.0;


	  // State feedback error
	  epv = v_ref_x_ - linearSpeedXMps_;
	  epw = w_ref_ - angularSpeedRads_;
	  // ROS_INFO("v_ref_x_=%5.2f, linearSpeedXMps_=%5.2f w_ref_=%5.2f angularSpeedRads_=%5.2f", v_ref_x_, linearSpeedXMps_, w_ref_, angularSpeedRads_);

	  // Compute state control actions
	  double uv= kpv * epv + kdv * (epv - epvant);
	  double uw= kpw * epw + kdw * (epw - epwant);
	  epvant = epv;
	  epwant = epw;

	  // Inverse kinematics
	  double dUl = uv - 0.5 * summit_xl_d_tracks_m_ * uw;
          // sign according to urdf (if wheel model is not symetric, should be inverted)
   	  double dUr = -(uv + 0.5 * summit_xl_d_tracks_m_ * uw);  

	  // Motor control actions
	  double limit = 40.0;
	  
	  //ROS_INFO("epv=%5.2f, epw=%5.2f ***  dUl=%5.2f  dUr=%5.2f", epv, epw, dUl, dUr);

	  // Axis are not reversed in the omni (swerve) configuration
      std_msgs::Float64 frw_ref_msg; 
      std_msgs::Float64 flw_ref_msg;
      std_msgs::Float64 blw_ref_msg;
      std_msgs::Float64 brw_ref_msg;
	  
      double k1 = 0.5; 
	  frw_ref_msg.data = saturation( k1 * dUr, -limit, limit);  
	  flw_ref_msg.data = saturation( k1 * dUl, -limit, limit);
      blw_ref_msg.data = saturation( k1 * dUl, -limit, limit);
	  brw_ref_msg.data = saturation( k1 * dUr, -limit, limit);
	  
	  ref_vel_frw_.publish( frw_ref_msg );
	  ref_vel_flw_.publish( flw_ref_msg );
	  ref_vel_blw_.publish( blw_ref_msg );
	  ref_vel_brw_.publish( brw_ref_msg );
	  }
	  */

/*
  // Depending on the robot configuration 
  if (active_kinematic_mode_ == MECANUM_STEERING) {

	  // Speed references for motor control
	  // double v_left_mps, v_right_mps;
	  double v_frw_mps, v_flw_mps, v_blw_mps, v_brw_mps; 
	  v_frw_mps = joint_state_.velocity[frw_vel_]  * (summit_xl_wheel_diameter_ / 2.0);
	  v_flw_mps = joint_state_.velocity[flw_vel_] * (summit_xl_wheel_diameter_ / 2.0);
	  v_blw_mps = joint_state_.velocity[blw_vel_] * (summit_xl_wheel_diameter_ / 2.0);
	  v_brw_mps = joint_state_.velocity[brw_vel_] * (summit_xl_wheel_diameter_ / 2.0);

	  double vx = v_ref_x_;
	  double vy = v_ref_y_;
	  double w = w_ref_;
	  double L = summit_xl_wheelbase_;   
	  double W = summit_xl_trackwidth_;
	  

	  double x1 = L/2.0; double y1 = - W/2.0;
	  double wx1 = v_ref_x_ - w_ref_ * y1;
	  double wy1 = v_ref_y_ + w_ref_ * x1;
	  double q1 = -sqrt( wx1*wx1 + wy1*wy1 );
	  double a1 = radnorm( atan2( wy1, wx1 ) );
	  double x2 = L/2.0; double y2 = W/2.0;
	  double wx2 = v_ref_x_ - w_ref_ * y2;
	  double wy2 = v_ref_y_ + w_ref_ * x2;
	  double q2 = sqrt( wx2*wx2 + wy2*wy2 );
	  double a2 = radnorm( atan2( wy2, wx2 ) );
	  double x3 = -L/2.0; double y3 = W/2.0;
	  double wx3 = v_ref_x_ - w_ref_ * y3;
	  double wy3 = v_ref_y_ + w_ref_ * x3;
	  double q3 = sqrt( wx3*wx3 + wy3*wy3 );
	  double a3 = radnorm( atan2( wy3, wx3 ) );
	  double x4 = -L/2.0; double y4 = -W/2.0;
	  double wx4 = v_ref_x_ - w_ref_ * y4;
	  double wy4 = v_ref_y_ + w_ref_ * x4;
	  double q4 = -sqrt( wx4*wx4 + wy4*wy4 );
	  double a4 = radnorm( atan2( wy4, wx4 ) );
	  
      //ROS_INFO("q1234=(%5.2f, %5.2f, %5.2f, %5.2f)   a1234=(%5.2f, %5.2f, %5.2f, %5.2f)", q1,q2,q3,q4, a1,a2,a3,a4);
	
	  // Motor control actions
	  double limit = 40.0;
	  
	  // Axis are not reversed in the omni (swerve) configuration
      std_msgs::Float64 frw_ref_vel_msg; 
      std_msgs::Float64 flw_ref_vel_msg;
      std_msgs::Float64 blw_ref_vel_msg;
      std_msgs::Float64 brw_ref_vel_msg;
	  frw_ref_vel_msg.data = saturation(-1.0 * (joint_state_.velocity[frw_vel_] - q1), -limit, limit);  
	  flw_ref_vel_msg.data = saturation(-1.0 * (joint_state_.velocity[flw_vel_] - q2), -limit, limit);
      blw_ref_vel_msg.data = saturation(-1.0 * (joint_state_.velocity[blw_vel_] - q3), -limit, limit);
	  brw_ref_vel_msg.data = saturation(-1.0 * (joint_state_.velocity[brw_vel_] - q4), -limit, limit);
	  ref_vel_frw_.publish( frw_ref_vel_msg );
	  ref_vel_flw_.publish( flw_ref_vel_msg );
	  ref_vel_blw_.publish( blw_ref_vel_msg );
	  ref_vel_brw_.publish( brw_ref_vel_msg );
	  std_msgs::Float64 frw_ref_pos_msg; 
      std_msgs::Float64 flw_ref_pos_msg;
      std_msgs::Float64 blw_ref_pos_msg;
      std_msgs::Float64 brw_ref_pos_msg;
	  frw_ref_pos_msg.data = a1; 
	  flw_ref_pos_msg.data = a2; 
	  blw_ref_pos_msg.data = a3; 
	  brw_ref_pos_msg.data = a4;
	  
	  ref_pos_frw_.publish( frw_ref_pos_msg);
	  ref_pos_flw_.publish( flw_ref_pos_msg);
	  ref_pos_blw_.publish( blw_ref_pos_msg);
	  ref_pos_brw_.publish( brw_ref_pos_msg);
	  
	  }
	  
	// Depending on the robot model 
	if (robot_model_ == "x_wam") {		
		// Position reference for the scissor mechanism
		static double scissor_ref_pos = 0.0;
		scissor_ref_pos  += (v_ref_z_ / desired_freq_);		
        std_msgs::Float64 scissor_ref_pos_msg;
        // joint_state_.position[scissor_pos_] not used, just setpoint 
        scissor_ref_pos_msg.data = saturation(scissor_ref_pos, 0.0, 0.5); 
        ref_pos_scissor_.publish( scissor_ref_pos_msg ); 
		}

     // PTZ 
	 std_msgs::Float64 pan_ref_pos_msg, tilt_ref_pos_msg;
     pan_ref_pos_msg.data = pos_ref_pan_;            //saturation( pos_ref_pan_, 0.0, 0.5); 
     ref_pos_pan_.publish( pan_ref_pos_msg );
     tilt_ref_pos_msg.data = pos_ref_tilt_;          //saturation( pos_ref_tilt_, 0.0, 0.5); 
     ref_pos_tilt_.publish( tilt_ref_pos_msg );
     */
}

// Update robot odometry depending on kinematic configuration
void UpdateOdometry()
{
  // Depending on the robot configuration 
  if (active_kinematic_mode_ == SKID_STEERING) {
	  
      // Compute Velocity (linearSpeedXMps_ computed in control
   	  robot_pose_vx_ = linearSpeedXMps_ *  cos(robot_pose_pa_);
      robot_pose_vy_ = linearSpeedXMps_ *  sin(robot_pose_pa_);
      }
		
  // Depending on the robot configuration 
  if (active_kinematic_mode_ == MECANUM_STEERING) {

      // Linear speed of each wheel
	  double v1, v2, v3, v4; 
	  v1 = joint_state_.velocity[frw_vel_]  * (summit_xl_wheel_diameter_ / 2.0);
	  v2 = joint_state_.velocity[flw_vel_] * (summit_xl_wheel_diameter_ / 2.0);
	  v3 = joint_state_.velocity[blw_vel_] * (summit_xl_wheel_diameter_ / 2.0);
	  v4 = joint_state_.velocity[brw_vel_] * (summit_xl_wheel_diameter_ / 2.0);
      double a1, a2, a3, a4;
      a1 = radnorm2( joint_state_.position[frw_pos_] );
      a2 = radnorm2( joint_state_.position[flw_pos_] );
      a3 = radnorm2( joint_state_.position[blw_pos_] );
      a4 = radnorm2( joint_state_.position[brw_pos_] );
      double v1x = -v1 * cos( a1 ); double v1y = -v1 * sin( a1 );
      double v2x = v2 * cos( a2 ); double v2y = v2 * sin( a2 );
      double v3x = v3 * cos( a3 ); double v3y = v3 * sin( a3 );
      double v4x = -v4 * cos( a4 ); double v4y = -v4 * sin( a4 );
      double C = (v4y + v1y) / 2.0;
      double B = (v2x + v1x) / 2.0;
      double D = (v2y + v3y) / 2.0;
      double A = (v3x + v4x) / 2.0;
      // double W = ( (B-A)/summit_xl_wheelbase_ ); // + (D-C)/summit_xl_trackwidth_ ) / 2.0;
      robot_pose_vx_ = (A+B) / 2.0;
      robot_pose_vy_ = (C+D) / 2.0;      	  
      }
      
      // Compute Position
      double fSamplePeriod = 1.0 / desired_freq_;
	  robot_pose_pa_ += ang_vel_z_ * fSamplePeriod;  // this velocity comes from IMU callback
      robot_pose_px_ += robot_pose_vx_ * fSamplePeriod;
      robot_pose_py_ += robot_pose_vy_ * fSamplePeriod;
      // ROS_INFO("Odom estimated x=%5.2f  y=%5.2f a=%5.2f", robot_pose_px_, robot_pose_py_, robot_pose_pa_);
}

// Publish robot odometry tf and topic depending 
void PublishOdometry()
{
	ros::Time current_time = ros::Time::now();
	
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = robot_pose_px_;
    odom_trans.transform.translation.y = robot_pose_py_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = orientation_x_;
	odom_trans.transform.rotation.y = orientation_y_;
	odom_trans.transform.rotation.z = orientation_z_;
	odom_trans.transform.rotation.w = orientation_w_;
	
    // send the transform over /tf
	// activate / deactivate with param
	// this tf in needed when not using robot_pose_ekf
    if (publish_odom_tf_) odom_broadcaster.sendTransform(odom_trans);  
        
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
	// Position
    odom.pose.pose.position.x = robot_pose_px_;
    odom.pose.pose.position.y = robot_pose_py_;
    odom.pose.pose.position.z = 0.0;
	// Orientation
    odom.pose.pose.orientation.x = orientation_x_;
	odom.pose.pose.orientation.y = orientation_y_;
	odom.pose.pose.orientation.z = orientation_z_;
	odom.pose.pose.orientation.w = orientation_w_;
	// Pose covariance
    for(int i = 0; i < 6; i++)
      		odom.pose.covariance[i*6+i] = 0.1;  // test 0.001

    //set the velocity
    odom.child_frame_id = "base_footprint";
	// Linear velocities
    odom.twist.twist.linear.x = robot_pose_vx_;
    odom.twist.twist.linear.y = robot_pose_vy_;
	odom.twist.twist.linear.z = 0.0;
	// Angular velocities
	odom.twist.twist.angular.x = ang_vel_x_;
	odom.twist.twist.angular.y = ang_vel_y_;
    odom.twist.twist.angular.z = ang_vel_z_;
	// Twist covariance
	for(int i = 0; i < 6; i++)
     		odom.twist.covariance[6*i+i] = 0.1;  // test 0.001

    //publish the message
    odom_pub_.publish(odom);
}

/// Controller stopping
void stopping()
{}


/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time_).toSec();

	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
		// TODO: Set Speed References to 0
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}


// Set the base velocity command
void setCommand(const geometry_msgs::Twist &cmd_vel)
{   
   v_ref_x_ = saturation(cmd_vel.linear.x, -10.0, 10.0);
   v_ref_y_ = saturation(cmd_vel.linear.y, -10.0, 10.0);        
   w_ref_ = saturation(cmd_vel.angular.z, -10.0, 10.0); // -10.0 10.0
   v_ref_z_ = saturation(cmd_vel.linear.z, -10.0, 10.0);
}

// CALLBACKS
// Service SetMode
bool srvCallback_SetMode(robotnik_msgs::set_mode::Request& request, robotnik_msgs::set_mode::Response& response)
{
  // Check if the selected mode is available or not
  // 1 - SKID STEERING
  // 2 - OMNIDRIVE 
  // (3 - SWERVE)

  ROS_INFO("SummitXLControllerClass::srvCallback_SetMode request.mode= %d", request.mode);
  if (request.mode == 1) {
	active_kinematic_mode_ = request.mode;
	return true;
	}
  if (request.mode == 2) {
	if (kinematic_modes_ == 2) {
		active_kinematic_mode_ = request.mode;
		return true;	
		}
	else return false;
	}
  return false;
}

// Service GetMode
bool srvCallback_GetMode(robotnik_msgs::get_mode::Request& request, robotnik_msgs::get_mode::Response& response)
{
  response.mode = active_kinematic_mode_;
  return true;	
}


// Service SetOdometry 
bool srvCallback_SetOdometry(robotnik_msgs::set_odometry::Request &request, robotnik_msgs::set_odometry::Response &response )
{
	// ROS_INFO("summit_xl_odometry::set_odometry: request -> x = %f, y = %f, a = %f", req.x, req.y, req.orientation);
	//robot_pose_px_ = req.x;
	//robot_pose_py_ = req.y;
	//robot_pose_pa_ = req.orientation;

	response.ret = true;
	return true;
}


// Topic command
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{	
  joint_state_ = *msg;
  read_state_ = true;
}

// Topic command
void commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // Safety check
  last_command_time_ = ros::Time::now();
  subs_command_freq->tick();			// For diagnostics

  base_vel_msg_ = *msg;
  this->setCommand(base_vel_msg_);
}

// Topic ptz command
void command_ptzCallback(const robotnik_msgs::ptzConstPtr& msg)
{
  pos_ref_pan_ += msg->pan / 180.0 * PI;
  pos_ref_tilt_ += msg->tilt / 180.0 * PI;
}

// Imu callback
void imuCallback( const sensor_msgs::Imu& imu_msg){

	orientation_x_ = imu_msg.orientation.x;
	orientation_y_ = imu_msg.orientation.y;
	orientation_z_ = imu_msg.orientation.z;
	orientation_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
	lin_acc_z_ = imu_msg.linear_acceleration.z;
}

double saturation(double u, double min, double max)
{
 if (u>max) u=max;
 if (u<min) u=min;
 return u; 
}

double radnorm( double value ) 
{
  while (value > PI) value -= PI;
  while (value < -PI) value += PI;
  return value;
}

double radnorm2( double value ) 
{
  while (value > 2.0*PI) value -= 2.0*PI;
  while (value < -2.0*PI) value += 2.0*PI;
  return value;
}

bool spin()
{
    ROS_INFO("summit_xl_robot_control::spin()");
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (starting() == 0)
      {
	    while(ros::ok() && node_handle_.ok()) {
          UpdateControl();
          UpdateOdometry();
          PublishOdometry();
          diagnostic_.update();
          ros::spinOnce();
	      r.sleep();
          }
	      ROS_INFO("END OF ros::ok() !!!");
      } else {
       // No need for diagnostic here since a broadcast occurs in start
       // when there is an error.
       usleep(1000000);
       ros::spinOnce();
      }
   }

   ROS_INFO("summit_xl_robot_control::spin() - end");
   return true;
}

}; // Class SummitXLControllerClass

int main(int argc, char** argv)
{
	ros::init(argc, argv, "summit_xl_robot_control");

	ros::NodeHandle n;		
  	SummitXLControllerClass sxlrc(n);

	
  	// ros::ServiceServer service = n.advertiseService("set_odometry", &summit_xl_node::set_odometry, &sxlc);
    sxlrc.spin();

	return (0);

	
}

