#include <summit_xl_localization/navsat_transform_new.h>

#include <robot_localization/filter_common.h>
#include <robot_localization/filter_utilities.h>
#include <robot_localization/navsat_conversions.h>
#include <robot_localization/ros_filter_utilities.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <XmlRpcException.h>

namespace RobotLocalization
{
NavSatTransformNew::NavSatTransformNew() :
    magneticDeclination_(0.0),
    utmOdomTfYaw_(0.0),
    yawOffset_(0.0),
    broadcastUtmTransform_(false),
    hasTransformOdom_(false),
    hasTransformGps_(false),
    hasTransformImu_(false),
    transformGood_(false),
    gpsUpdated_(false),
    odomUpdated_(false),
    publishGps_(false),
    zeroAltitude_(false),
    worldFrameId_("odom"),
    baseLinkFrameId_("base_link"),
    utmZone_(""),
    tfListener_(tfBuffer_),  
    travelledDistance_(-1.0),
    localOdomTopic_("odometry/filtered"),
    hasToPublish_(false),
    previous_x_(0.0),
    previous_y_(0.0)
{
  latestUtmCovariance_.resize(POSE_SIZE, POSE_SIZE);
}

  NavSatTransformNew::~NavSatTransformNew() {}

  void NavSatTransformNew::localOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    double dist_x=fabs(msg->pose.pose.position.x-previous_x_);
    double dist_y=fabs(msg->pose.pose.position.y-previous_y_);

    double distance=sqrt(dist_x*dist_x+dist_y*dist_y);

    if(distance>travelledDistance_)
      {
	hasToPublish_ = true;
	previous_x_=msg->pose.pose.position.x;
	previous_y_=msg->pose.pose.position.y;
      }
  }

  void NavSatTransformNew::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if(!transformGood_)
    {
      setTransformOdometry(msg);
    }

    tf2::fromMsg(msg->pose.pose, latestWorldPose_);
    odomUpdateTime_ = msg->header.stamp;
    odomUpdated_ = true;
  }

void NavSatTransformNew::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{

    // Make sure the GPS data is usable
    bool goodGps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                    !std::isnan(msg->altitude) &&
                    !std::isnan(msg->latitude) &&
                    !std::isnan(msg->longitude));

    if(goodGps)
    {
      // If we haven't computed the transform yet, then
      // store this message as the initial GPS data to use
      if(!transformGood_)
      {
        setTransformGps(msg);
      }

      double utmX = 0;
      double utmY = 0;
      std::string utmZoneTmp;
      NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZoneTmp);
      latestUtmPose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
      latestUtmCovariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latestUtmCovariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
        }
      }

      gpsUpdateTime_ = msg->header.stamp;
      gpsUpdated_ = true;
    }
  }

void NavSatTransformNew::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
{
  double utmX = 0;
  double utmY = 0;
  NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZone_);
  
  ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " << msg->longitude << ", " << msg->altitude << ")");
  ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utmX << ", " << utmY << ")");
  
  transformUtmPose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
  transformUtmPose_.setRotation(tf2::Quaternion::getIdentity());
  hasTransformGps_ = true;
}

void NavSatTransformNew::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  tf2::fromMsg(msg->pose.pose, transformWorldPose_);
  worldFrameId_ = msg->header.frame_id;
  baseLinkFrameId_ = msg->child_frame_id;
  hasTransformOdom_ = true;

  ROS_INFO_STREAM("Initial odometry position is (" << std::fixed << 
		  transformWorldPose_.getOrigin().getX() << ", " << 
		  transformWorldPose_.getOrigin().getY() << ", " << 
		  transformWorldPose_.getOrigin().getZ() << ")");
}

void NavSatTransformNew::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  // We need the baseLinkFrameId_ from the odometry message, so
  // we need to wait until we receive it.
  if(hasTransformOdom_)
    {
      /* This method only gets called if we don't yet have the
       * IMU data (the subscriber gets shut down once we compute
       * the transform), so we can assumed that every IMU message
       * that comes here is meant to be used for that purpose. */
      tf2::fromMsg(msg->orientation, transformOrientation_);
      
      // Correct for the IMU's orientation w.r.t. base_link
      tf2::Transform targetFrameTrans;
      bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                                  baseLinkFrameId_,
                                                                  msg->header.frame_id,
                                                                  msg->header.stamp,
                                                                  targetFrameTrans);

      if(canTransform)
      {
        double rollOffset = 0;
        double pitchOffset = 0;
        double yawOffset = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        RosFilterUtilities::quatToRPY(targetFrameTrans.getRotation(), rollOffset, pitchOffset, yawOffset);
        RosFilterUtilities::quatToRPY(transformOrientation_, roll, pitch, yaw);

        ROS_DEBUG_STREAM("Initial orientation roll, pitch, yaw is (" <<
                         roll << ", " << pitch << ", " << yaw << ")");

        // Apply the offset (making sure to bound them), and throw them in a vector
        tf2::Vector3 rpyAngles(FilterUtilities::clampRotation(roll - rollOffset),
                               FilterUtilities::clampRotation(pitch - pitchOffset),
                               FilterUtilities::clampRotation(yaw - yawOffset));

        // Now we need to rotate the roll and pitch by the yaw offset value.
        // Imagine a case where an IMU is mounted facing sideways. In that case
        // pitch for the IMU's world frame is roll for the robot.
        tf2::Matrix3x3 mat;
        mat.setRPY(0.0, 0.0, yawOffset);
        rpyAngles = mat * rpyAngles;
        transformOrientation_.setRPY(rpyAngles.getX(), rpyAngles.getY(), rpyAngles.getZ());

        ROS_DEBUG_STREAM("Initial corrected orientation roll, pitch, yaw is (" <<
                         rpyAngles.getX() << ", " << rpyAngles.getY() << ", " << rpyAngles.getZ() << ")");

        hasTransformImu_ = true;
      }
    }
}

void NavSatTransformNew::computeTransform()
{
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the data we need
    if(!transformGood_ && hasTransformOdom_ && hasTransformGps_ && hasTransformImu_)
    {
      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf2::Matrix3x3 mat(transformOrientation_);

      // Convert to RPY
      double imuRoll;
      double imuPitch;
      double imuYaw;
      mat.getRPY(imuRoll, imuPitch, imuYaw);

      /* The IMU's heading was likely originally reported w.r.t. magnetic north.
       * However, all the nodes in robot_localization assume that orientation data,
       * including that reported by IMUs, is reported in an ENU frame, with a 0 yaw
       * value being reported when facing east and increasing counter-clockwise (i.e.,
       * towards north). Conveniently, this aligns with the UTM grid, where X is east
       * and Y is north. However, we have two additional considerations:
       *   1. The IMU may have its non-ENU frame data transformed to ENU, but there's
       *      a possibility that its data has not been corrected for magnetic
       *      declination. We need to account for this. A positive magnetic
       *      declination is counter-clockwise in an ENU frame. Therefore, if
       *      we have a magnetic declination of N radians, then when the sensor
       *      is facing a heading of N, it reports 0. Therefore, we need to add
       *      the declination angle.
       *   2. To account for any other offsets that may not be accounted for by the
       *      IMU driver or any interim processing node, we expose a yaw offset that
       *      lets users work with navsat_transform_node.
       */
      imuYaw += (magneticDeclination_ + yawOffset_);

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magneticDeclination_ <<
                      " and user-specified offset of " << yawOffset_ << ". Transform heading factor is now " << imuYaw);

      // Convert to tf-friendly structures
      tf2::Quaternion imuQuat;
      imuQuat.setRPY(0.0, 0.0, imuYaw);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf2::Transform utmPoseWithOrientation;
      utmPoseWithOrientation.setOrigin(transformUtmPose_.getOrigin());
      utmPoseWithOrientation.setRotation(imuQuat);
      utmWorldTransform_.mult(transformWorldPose_, utmPoseWithOrientation.inverse());

      utmWorldTransInverse_ = utmWorldTransform_.inverse();

      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      mat.setRotation(latestWorldPose_.getRotation());
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("Transform world frame pose is: " << std::fixed <<
                      "\nPosition: (" << transformWorldPose_.getOrigin().getX() << ", " <<
                                         transformWorldPose_.getOrigin().getY() << ", " <<
                                         transformWorldPose_.getOrigin().getZ() << ")" <<
                      "\nOrientation: (" << roll << ", " <<
                                            pitch << ", " <<
                                            yaw << ")");

      mat.setRotation(utmWorldTransform_.getRotation());
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("World frame->utm transform is " << std::fixed <<
                       "\nPosition: (" << utmWorldTransform_.getOrigin().getX() << ", " <<
                                          utmWorldTransform_.getOrigin().getY() << ", " <<
                                          utmWorldTransform_.getOrigin().getZ() << ")" <<
                       "\nOrientation: (" << roll << ", " <<
                                             pitch << ", " <<
                                             yaw << ")");

      transformGood_ = true;

      // Send out the (static) UTM transform in case anyone else would like to use it.
      if(broadcastUtmTransform_)
      {
        geometry_msgs::TransformStamped utmTransformStamped;
        utmTransformStamped.header.stamp = ros::Time::now();
        utmTransformStamped.header.frame_id = worldFrameId_;
        utmTransformStamped.child_frame_id = "utm";
        utmTransformStamped.transform = tf2::toMsg(utmWorldTransform_);
        utmBroadcaster_.sendTransform(utmTransformStamped);
      }
    }
}

bool NavSatTransformNew::prepareGpsOdometry(nav_msgs::Odometry &gpsOdom)
{
  bool newData = false;

  if(transformGood_ && gpsUpdated_)
    {
      tf2::Transform transformedUtm;

      transformedUtm.mult(utmWorldTransform_, latestUtmPose_);
      transformedUtm.setRotation(tf2::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utmWorldTransform_.getRotation());
      Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
      rot6d.setIdentity();

      for(size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot6d(rInd, 0) = rot.getRow(rInd).getX();
        rot6d(rInd, 1) = rot.getRow(rInd).getY();
        rot6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latestUtmCovariance_ = rot6d * latestUtmCovariance_.eval() * rot6d.transpose();

      // Now fill out the message. Set the orientation to the identity.
      tf2::toMsg(transformedUtm, gpsOdom.pose.pose);
      gpsOdom.pose.pose.position.z = (zeroAltitude_ ? 0.0 : gpsOdom.pose.pose.position.z);

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          gpsOdom.pose.covariance[POSE_SIZE * i + j] = latestUtmCovariance_(i, j);
        }
      }

      gpsOdom.header.frame_id = worldFrameId_;
      gpsOdom.header.stamp = gpsUpdateTime_;

      // Mark this GPS as used
      gpsUpdated_ = false;
      newData = true;
    }

    return newData;
}
 
void NavSatTransformNew::run()
{
  ros::Time::init();

    double frequency = 10.0;
    double delay = 0.0;

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
   // Load the parameters we need
    nhPriv.getParam("magnetic_declination_radians", magneticDeclination_);
    nhPriv.param("yaw_offset", yawOffset_, 0.0);
    nhPriv.param("broadcast_utm_transform", broadcastUtmTransform_, false);
    nhPriv.param("zero_altitude", zeroAltitude_, false);
    nhPriv.param("publish_filtered_gps", publishGps_, false);
    nhPriv.param("frequency", frequency, 10.0);
    nhPriv.param("delay", delay, 0.0);
    
    //NEW PARAMETERS NEEDED
    nhPriv.param("travelled_distance",travelledDistance_,-1.0);
    nhPriv.param<std::string>("localOdomTopic",localOdomTopic_,"");

    ros::Subscriber odomSub = nh.subscribe("odometry/filtered", 1, &NavSatTransformNew::odomCallback, this);
    ros::Subscriber gpsSub = nh.subscribe("gps/fix", 1, &NavSatTransformNew::gpsFixCallback, this);
    ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("imu/data", 1, &NavSatTransformNew::imuCallback, this);

    ros::Subscriber localOdomSub = nh.subscribe(localOdomTopic_, 1,&NavSatTransformNew::localOdomCallback, this);
    
    ros::Publisher gpsOdomPub = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);
    ros::Publisher filteredGpsPub;
    
    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    ros::Duration startDelay(delay);
    startDelay.sleep();

    ros::Rate rate(frequency);
    while(ros::ok())
    {
      ros::spinOnce();

      if(!transformGood_)
      {
        computeTransform();

        if(transformGood_)
        {
          // Once we have the transform, we don't need the IMU
          imuSub.shutdown();
        }
      }
      else
      {
        if(hasToPublish_)
	  {
	    nav_msgs::Odometry gpsOdom;
	    if(prepareGpsOdometry(gpsOdom))
	      {
		gpsOdomPub.publish(gpsOdom);
		ROS_INFO("Publish GPS odometry");
	      }
	    hasToPublish_=false;
	  }
      }
      rate.sleep();
    }
  }
  
}
