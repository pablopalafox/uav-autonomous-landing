#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <robotnik_msgs/set_odometry.h>  //! To reset both the ekf_localization_node and the summit_xl_controller node
#include <mavros_msgs/CommandLong.h>     //! To calibrate PIXHAWK sensors
#include <robot_localization/SetPose.h>
#include <std_srvs/Trigger.h>

#include <mavros_msgs/CommandLong.h>

#define MAVLINK_CALIBRATION_COMMAND 241

/* Provide tools and ROS Services for the Summit series mobile robots dead-reckoning. Namely:
   - /reset_odometry. ROS Service to reset both the state estimation of the ekf_localization_node and 
     the odometry estimate inside the summit_xl_controller node.
     NOTE: If you are not using the ekf_localization_node, you need to call the 
     /set_odometry ROS Service instead 

   - /calibrate_imu_gyro, /calibrate_imu_acc, /calibrate_imu_mag, /calibrate_offset. ROS Services
     to launch the calibration of PIXHAWK sensors. The feedback of the calibration process is provided
     by the mavros_node (if the terminal in which the mavros_node is launched is not displayed, you won't
     get any feedback back)
 */

using namespace std;

class LocalizationUtils{
public:
  LocalizationUtils(ros::NodeHandle nh);

  void odomCallback (const nav_msgs::OdometryConstPtr& odom);
  void imuCallback (const sensor_msgs::ImuConstPtr& msg);
  bool resetOdomCallback(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &resp);

  bool calGyroCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool calAccCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool calMagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool calOffCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::ServiceClient set_odometry_;
  ros::ServiceClient set_pose_;
  ros::ServiceServer reset_odom_;
  
  ros::ServiceServer calib_gyro_srv_;
  ros::ServiceServer calib_acc_srv_;
  ros::ServiceServer calib_mag_srv_;
  ros::ServiceServer calib_off_srv_;
  ros::ServiceClient mavcmd_client_;
  
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
  
  string odom_topic_;
  string imu_topic_;
  string imu_output_topic_;

  //FLAGS
  //If true, the node subscribes to sensor_msgs/Imu messages and republishes them dynamically adaptating the orientation covariance matrix accordingly to the state of motion of the robot
  string dynamic_imu_covariance_;
  bool in_motion_;
  float angular_vel_;
};

LocalizationUtils::LocalizationUtils(ros::NodeHandle nh): nh_(nh), private_nh_("~"), 
				    			  in_motion_(false), angular_vel_(0.0)
{
  private_nh_.param<string>("odom_topic", odom_topic_, "/odom");
  private_nh_.param<string>("imu_topic", imu_topic_, "/mavros/imu/data");
  private_nh_.param<string>("dynamic_imu_covariance", dynamic_imu_covariance_, "false");
  private_nh_.param<string>("imu_outpur_topic", imu_output_topic_, "/mavros/imu/data_adapted");

  if (dynamic_imu_covariance_ == "true"){
	ROS_INFO("[LocalizationUtils]: using dynamic Imu covariance");
  	imu_sub_=nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 1, &LocalizationUtils::imuCallback, this);
  	odom_sub_=nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &LocalizationUtils::odomCallback, this);
  	imu_pub_=nh_.advertise<sensor_msgs::Imu>(imu_output_topic_,10);
  }

  mavcmd_client_=nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  calib_gyro_srv_= nh_.advertiseService("calibrate_imu_gyro", &LocalizationUtils::calGyroCallback, this);
  calib_acc_srv_= nh_.advertiseService("calibrate_imu_acc", &LocalizationUtils::calAccCallback, this);
  calib_mag_srv_= nh_.advertiseService("calibrate_imu_mag", &LocalizationUtils::calMagCallback, this);
  calib_off_srv_= nh_.advertiseService("calibrate_imu_off", &LocalizationUtils::calOffCallback, this);
  
  set_odometry_ = nh_.serviceClient<robotnik_msgs::set_odometry>("/set_odometry");
  set_pose_ = nh_.serviceClient<robot_localization::SetPose>("/set_pose");
  reset_odom_=nh_.advertiseService("/reset_odometry",&LocalizationUtils::resetOdomCallback,this);
}

bool LocalizationUtils::calGyroCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){

  mavros_msgs::CommandLong::Request req_mavros;
  mavros_msgs::CommandLong::Response resp_mavros;

  req_mavros.command=MAVLINK_CALIBRATION_COMMAND;
  req_mavros.confirmation=1;
  req_mavros.param1=1;
  req_mavros.param2=0;
  req_mavros.param3=0;
  req_mavros.param4=0;
  req_mavros.param5=0;
  req_mavros.param6=0;    
  req_mavros.param7=0;	

  bool success_odom=mavcmd_client_.call(req_mavros,resp_mavros);

  resp.success=resp_mavros.success;
  //resp_mavros.result is the raw result returned by COMMAND_ACK
  //TODO save resp_mavros.result in resp.message (conversion from unsigned string to int)
  
  return true;
}

bool LocalizationUtils::calAccCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
  
  mavros_msgs::CommandLong::Request req_mavros;
  mavros_msgs::CommandLong::Response resp_mavros;

  req_mavros.command=MAVLINK_CALIBRATION_COMMAND;
  req_mavros.confirmation=1;
  req_mavros.param1=0;
  req_mavros.param2=0;
  req_mavros.param3=0;
  req_mavros.param4=0;
  req_mavros.param5=1;
  req_mavros.param6=0;    
  req_mavros.param7=0;	

  bool success_odom=mavcmd_client_.call(req_mavros,resp_mavros);

  resp.success=resp_mavros.success;
  return true;
}

bool LocalizationUtils::calMagCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
  
  mavros_msgs::CommandLong::Request req_mavros;
  mavros_msgs::CommandLong::Response resp_mavros;

  req_mavros.command=MAVLINK_CALIBRATION_COMMAND;
  req_mavros.confirmation=1;
  req_mavros.param1=0;
  req_mavros.param2=1;
  req_mavros.param3=0;
  req_mavros.param4=0;
  req_mavros.param5=0;
  req_mavros.param6=0;    
  req_mavros.param7=0;	

  bool success_odom=mavcmd_client_.call(req_mavros,resp_mavros);

  resp.success=resp_mavros.success; 
  return true;
}

bool LocalizationUtils::calOffCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
  
  mavros_msgs::CommandLong::Request req_mavros;
  mavros_msgs::CommandLong::Response resp_mavros;

  req_mavros.command=MAVLINK_CALIBRATION_COMMAND;
  req_mavros.confirmation=1;
  req_mavros.param1=0;
  req_mavros.param2=0;
  req_mavros.param3=0;
  req_mavros.param4=0;
  req_mavros.param5=2;
  req_mavros.param6=0;    
  req_mavros.param7=0;	

  bool success_odom=mavcmd_client_.call(req_mavros,resp_mavros);

  resp.success=resp_mavros.success; 
  return true;
}

void LocalizationUtils::imuCallback (const sensor_msgs::ImuConstPtr& imu)
{
  sensor_msgs::Imu msg=*imu;
  /*  if(in_motion_)
    {
      msg.orientation_covariance[8]=1e9;
      ROS_INFO("In_motion, covariance 1e9");
    }
  else
    {
      msg.orientation_covariance[8]=1e-9;
      ROS_INFO("In_motion, covariance 1e-9");
      }*/

  double scaling_factor_=1e6;
  msg.orientation_covariance[8]=(angular_vel_>0.8) ? 1e9 : angular_vel_*1e-9*scaling_factor_;
  
  imu_pub_.publish(msg);
}

void LocalizationUtils::odomCallback (const nav_msgs::OdometryConstPtr& odom)
{
  in_motion_=(fabs(odom->twist.twist.linear.x)>0.01 || fabs(odom->twist.twist.linear.y)>0.01 ||
	      fabs(odom->twist.twist.angular.z)>0.005) ? true : false;

  angular_vel_=odom->twist.twist.angular.z;
}

bool LocalizationUtils::resetOdomCallback(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &resp)
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id ="odom";
  msg.pose.pose.position.x=req.x;
  msg.pose.pose.position.y=req.y;
  msg.pose.pose.position.z=req.z;

  msg.pose.pose.orientation.x=0.0;
  msg.pose.pose.orientation.y=0.0;
  msg.pose.pose.orientation.z=0.0;
  msg.pose.pose.orientation.w=1.0;

  robotnik_msgs::set_odometry::Request req_odom;
  robotnik_msgs::set_odometry::Response resp_odom;

  robot_localization::SetPose::Request req_rl;
  robot_localization::SetPose::Response resp_rl;

  req_odom=req;
  req_rl.pose=msg;
  bool success_odom=set_odometry_.call(req_odom,resp_odom);
  bool success_rl=set_pose_.call(req_rl,resp_rl);

  //TODO: sometimes the service needs to be called twice to reset the ekf_localization_node output
  resp.ret=(success_odom && success_rl);
  
  return true;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_helper_node");
  ros::NodeHandle nh;

  LocalizationUtils utils(nh);
  ros::spin();

  return 0;
}
