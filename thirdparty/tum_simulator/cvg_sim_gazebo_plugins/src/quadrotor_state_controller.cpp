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
* Modified on: Apr 28, 2019
* Pablo R. Palafox
*
*
*/


#include <hector_quadrotor_controller/quadrotor_state_controller.h>
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

#include <cmath>

namespace gazebo {

GazeboQuadrotorStateController::GazeboQuadrotorStateController()
{
  robot_current_state = INITIALIZE_MODEL;
  m_isFlying          = false;
  m_takeoff           = false; 
  m_drainBattery      = true;
  m_batteryPercentage = 100;
  m_maxFlightTime     = 1200;
  m_timeAfterTakeOff  = 0;
  m_selected_cam_num  = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorStateController::~GazeboQuadrotorStateController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorStateController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString() + "/";

  if (!_sdf->HasElement("topicName"))
    velocity_topic_ = "/ardrone/cmd_vel";
  else
    velocity_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (!_sdf->HasElement("takeoffTopic"))
    takeoff_topic_ = "/ardrone/takeoff";
  else
    takeoff_topic_ = _sdf->GetElement("takeoffTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("/ardrone/land"))
    land_topic_ = "/ardrone/land";
  else
    land_topic_ = _sdf->GetElement("landTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("resetTopic"))
    reset_topic_ = "/ardrone/reset";
  else
    reset_topic_ = _sdf->GetElement("resetTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("navdataTopic"))
    navdata_topic_ = "/ardrone/navdata";
  else
    navdata_topic_ = _sdf->GetElement("navdataTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("sonarTopic"))
    sonar_topic_.clear();
  else
    sonar_topic_ = _sdf->GetElement("sonarTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("stateTopic"))
    state_topic_.clear();
  else
    state_topic_ = _sdf->GetElement("stateTopic")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_));
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // subscribe command: velocity control command
  if (!velocity_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      velocity_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::VelocityCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: take off command
  if (!takeoff_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      takeoff_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::TakeoffCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    takeoff_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: take off command
  if (!land_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      land_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::LandCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    land_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: take off command
  if (!reset_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      reset_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::ResetCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    reset_subscriber_ = node_handle_->subscribe(ops);
  }

    m_navdataPub = node_handle_->advertise< ardrone_autonomy::Navdata >( navdata_topic_ , 25 );


  // subscribe imu
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_state_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  }

  // subscribe sonar senor info
  if (!sonar_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Range>(
      sonar_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::SonarCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    sonar_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_state_controller", "Using sonar information on topic %s as source of altitude.", sonar_topic_.c_str());
  }

  // subscribe state
  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::StateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_state_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
  }


  // for camera control
  // switch camera server
  std::string toggleCam_topic  = "ardrone/togglecam";
  ros::AdvertiseServiceOptions toggleCam_ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    toggleCam_topic,
    boost::bind(&GazeboQuadrotorStateController::toggleCamCallback, this, _1,_2),
    ros::VoidPtr(),
    &callback_queue_);

  toggleCam_service = node_handle_->advertiseService(toggleCam_ops);

  // camera image data
  std::string cam_out_topic  = "/ardrone/image_raw";
  std::string cam_front_in_topic = "/ardrone/front/image_raw";
  std::string cam_bottom_in_topic = "/ardrone/bottom/image_raw";
  std::string in_transport = "raw";

  camera_it_ = new image_transport::ImageTransport(*node_handle_);
  camera_publisher_ = camera_it_->advertise(cam_out_topic, 1);

  camera_front_subscriber_ = camera_it_->subscribe(
    cam_front_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraFrontCallback, this, _1),
    ros::VoidPtr(), in_transport);

  camera_bottom_subscriber_ = camera_it_->subscribe(
    cam_bottom_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraBottomCallback, this, _1),
    ros::VoidPtr(), in_transport);

  // camera image data
  std::string cam_info_out_topic  = "/ardrone/camera_info";
  std::string cam_info_front_in_topic = "/ardrone/front/camera_info";
  std::string cam_info_bottom_in_topic = "/ardrone/bottom/camera_info";

  camera_info_publisher_ = node_handle_->advertise<sensor_msgs::CameraInfo>(cam_info_out_topic,1);

  ros::SubscribeOptions cam_info_front_ops = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(
    cam_info_front_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraInfoFrontCallback, this, _1),
    ros::VoidPtr(), &callback_queue_);
  camera_info_front_subscriber_ = node_handle_->subscribe(cam_info_front_ops);

  ros::SubscribeOptions cam_info_bottom_ops = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(
    cam_info_bottom_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraInfoBottomCallback, this, _1),
    ros::VoidPtr(), &callback_queue_);
  camera_info_bottom_subscriber_ = node_handle_->subscribe(cam_info_bottom_ops);

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorStateController::CallbackQueueThread,this ) );


  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboQuadrotorStateController::Update, this));

  robot_current_state = LANDED_MODEL;
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorStateController::VelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
{
  velocity_command_ = *velocity;
}

void GazeboQuadrotorStateController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void GazeboQuadrotorStateController::SonarCallback(const sensor_msgs::RangeConstPtr& sonar_info)
{
  robot_altitude = sonar_info->range;
}


void GazeboQuadrotorStateController::StateCallback(const nav_msgs::OdometryConstPtr& state)
{
  math::Vector3 velocity1(velocity);

  if (imu_topic_.empty()) {
    pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
    euler = pose.rot.GetAsEuler();
    angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0) {
    acceleration = (velocity - velocity1) / dt;
  } else {
    acceleration.Set();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorStateController::Update()
{
  // Get new commands/state
  callback_queue_.callAvailable();

  // Get simulator time
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  // Update rate is 200/per second
  if (dt < 0.005) return;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  if (imu_topic_.empty()) {
    pose = link->GetWorldPose();
    angular_velocity = link->GetWorldAngularVel();
    euler = pose.rot.GetAsEuler();
  }
  if (state_topic_.empty()) {
    acceleration = (link->GetWorldLinearVel() - velocity) / dt;
    velocity = link->GetWorldLinearVel();
  }

  // Rotate vectors to coordinate frames relevant for control
  math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
  math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

  // process robot operation information
  if((m_takeoff)&&(robot_current_state == LANDED_MODEL))
  {
    m_timeAfterTakeOff = 0;
    m_takeoff = false;
    robot_current_state = TAKINGOFF_MODEL;
  }
  else if(robot_current_state == TAKINGOFF_MODEL)
  {
    // take off phase need more power
    if (!sonar_topic_.empty())
    {
      if(robot_altitude > 0.05)
      {
        robot_current_state = FLYING_MODEL;
      }
    }
    if(!m_isFlying)
    {
      m_timeAfterTakeOff = 0;
      robot_current_state = LANDING_MODEL;
    }
  }
  else if((robot_current_state == FLYING_MODEL)||(robot_current_state == TO_FIX_POINT_MODEL))
  {
    if(m_isFlying == false)
    {
      m_timeAfterTakeOff = 0;
      robot_current_state = LANDING_MODEL;
    }
  }
  else if(robot_current_state == LANDING_MODEL)
  {
    if (!sonar_topic_.empty())
    {
      m_timeAfterTakeOff += dt;
      if((robot_altitude < 0.06))
      {
        robot_current_state = LANDED_MODEL;
      }
    }
    else
    {
      m_timeAfterTakeOff += dt;
      if(m_timeAfterTakeOff > 1.0)
      {
        robot_current_state = LANDED_MODEL;
      }
    }

    if(m_isFlying == true)
    {
      m_timeAfterTakeOff = 0;
      m_takeoff = false;
      robot_current_state = TAKINGOFF_MODEL;
    }
  }

  if( ((robot_current_state != LANDED_MODEL) || m_isFlying) && m_drainBattery )
    m_batteryPercentage -= dt / m_maxFlightTime * 100.;

  ardrone_autonomy::Navdata navdata;
  navdata.batteryPercent = m_batteryPercentage;
  navdata.rotX = pose.rot.GetRoll() / M_PI * 180.;
  navdata.rotY = pose.rot.GetPitch() / M_PI * 180.;
  navdata.rotZ = pose.rot.GetYaw() / M_PI * 180.;
  if (!sonar_topic_.empty()) {
    navdata.altd = int(robot_altitude*1000);
  }
  else {
    navdata.altd = pose.pos.z * 1000.f;
  }
  navdata.vx = 1000*velocity_xy.x;
  navdata.vy = 1000*velocity_xy.y;
  navdata.vz = 1000*velocity_xy.z;
  navdata.ax = acceleration_xy.x/10;
  navdata.ay = acceleration_xy.y/10;
  navdata.az = acceleration_xy.z/10 + 1;
  navdata.tm = ros::Time::now().toSec()*1000000; // FIXME what is the real drone sending here?


  navdata.header.stamp = ros::Time::now();
  navdata.header.frame_id = "/ardrone/base_link";
  navdata.state = robot_current_state;
  navdata.magX = 0;
  navdata.magY = 0;
  navdata.magZ = 0;
  navdata.pressure = 0;
  navdata.temp = 0;
  navdata.wind_speed = 0.0;
  navdata.wind_angle = 0.0;
  navdata.wind_comp_angle = 0.0;
  navdata.tags_count = 0;
//  navdata.tags_type
//  navdata.tags_xc
//  navdata.tags_yc
//  navdata.tags_width
//  navdata.tags_height
//  navdata.tags_orientation
//  navdata.tags_distance
   
//  filter for sensor information
//  filter_rate = 0.1;
//  navdata.rotX = navdata.rotX*filter_rate + (1-filter_rate)*last_navdata.rotX;
//  navdata.rotY = navdata.rotY*filter_rate + (1-filter_rate)*last_navdata.rotY;
//  navdata.rotZ = navdata.rotZ*filter_rate + (1-filter_rate)*last_navdata.rotZ;
//  navdata.altd = navdata.altd*filter_rate + (1-filter_rate)*last_navdata.altd;
//  navdata.vx = navdata.vx*filter_rate + (1-filter_rate)*last_navdata.vx;
//  navdata.vy = navdata.vy*filter_rate + (1-filter_rate)*last_navdata.vy;
//  navdata.vz = navdata.vz*filter_rate + (1-filter_rate)*last_navdata.vz;
//  navdata.ax = navdata.ax*filter_rate + (1-filter_rate)*last_navdata.ax;
//  navdata.ay = navdata.ay*filter_rate + (1-filter_rate)*last_navdata.ay;
//  navdata.az = navdata.az*filter_rate + (1-filter_rate)*last_navdata.az;
//  last_navdata = navdata;

  m_navdataPub.publish( navdata );

  // save last time stamp
  last_time = sim_time;

}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorStateController::Reset()
{
  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////
// controller callback
void GazeboQuadrotorStateController::TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
{
  if(robot_current_state == LANDED_MODEL)
  {
    m_isFlying = true;
    m_takeoff = true;
    m_batteryPercentage = 100.;
    ROS_INFO("%s","\nQuadrotor takes off!!");
  }
  else if(robot_current_state == LANDING_MODEL)
  {
    m_isFlying = true;
    m_takeoff = true;
    ROS_INFO("%s","\nQuadrotor takes off!!");
  }
}

void GazeboQuadrotorStateController::LandCallback(const std_msgs::EmptyConstPtr& msg)
{
  if((robot_current_state == FLYING_MODEL)||(robot_current_state == TO_FIX_POINT_MODEL)||(robot_current_state == TAKINGOFF_MODEL))
  {
    m_isFlying = false;
    m_takeoff = false;
    ROS_INFO("%s","\nQuadrotor lands!!");
  }

}

void GazeboQuadrotorStateController::ResetCallback(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO("%s","\nReset quadrotor!!");
}

bool GazeboQuadrotorStateController::toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if(m_selected_cam_num==0)
    m_selected_cam_num = 1;
  else if(m_selected_cam_num==1)
    m_selected_cam_num = 0;

  ROS_INFO("\nSetting camera channel to : %d.\n", m_selected_cam_num);
  return true;
}

void GazeboQuadrotorStateController::CameraFrontCallback(const sensor_msgs::ImageConstPtr& image)
{
  if(m_selected_cam_num==0)
    camera_publisher_.publish(image);
}

void GazeboQuadrotorStateController::CameraBottomCallback(const sensor_msgs::ImageConstPtr& image)
{
  if(m_selected_cam_num==1)
    camera_publisher_.publish(image);
}

void GazeboQuadrotorStateController::CameraInfoFrontCallback(const sensor_msgs::CameraInfoConstPtr&  image_info)
{
  if(m_selected_cam_num==0)
    camera_info_publisher_.publish(image_info);
}

void GazeboQuadrotorStateController::CameraInfoBottomCallback(const sensor_msgs::CameraInfoConstPtr&  image_info)
{
  if(m_selected_cam_num==1)
    camera_info_publisher_.publish(image_info);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorStateController)

} // namespace gazebo
