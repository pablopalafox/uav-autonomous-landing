#include <math.h>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/PoseStamped.h>

#include <takeoff/GroundtruthAltitude.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#define PI 3.1415

class RobotStateBroadcaster {
public:
    RobotStateBroadcaster();
private:
    ros::NodeHandle nh_;

    ros::ServiceClient get_model_state_;

    gazebo_msgs::GetModelState gms_ardrone_;
    gazebo_msgs::GetModelState gms_summit_;

    // tf::TransformBroadcaster br_;

    double x_ardrone_, y_ardrone_, z_ardrone_;
    double x_summit_, y_summit_, z_summit_;

    ros::Publisher altitude_pub_;
    ros::Publisher quadrotor_position_pub_;
    ros::Publisher summit_position_pub_;
};

RobotStateBroadcaster::RobotStateBroadcaster() {


    gms_ardrone_.request.model_name = "ardrone";
    gms_summit_.request.model_name = "summit_xl";

    // gazebo services
    ros::service::waitForService("/gazebo/get_model_state");
    get_model_state_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // publisher
    altitude_pub_ = nh_.advertise<takeoff::GroundtruthAltitude>("/ardrone/groundtruth_altitude", 1);
    quadrotor_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/groundtruth/ardrone", 1);
    summit_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/groundtruth/summit", 1);

    while (nh_.ok()) {

        // quadrotor
        if (!get_model_state_.call(gms_ardrone_)) {
            ROS_ERROR("RobotStateBroadcaster::Failed to call service ardrone");
        }
        if (!gms_ardrone_.response.success) {
            ROS_ERROR("RobotStateBroadcaster::Response to service ardrone failed");
        }
        else {
            x_ardrone_ = gms_ardrone_.response.pose.position.x;
            y_ardrone_ = gms_ardrone_.response.pose.position.y;
            z_ardrone_ = gms_ardrone_.response.pose.position.z;
            ROS_INFO("Ardrone position in world = (%.4f, %.4f, %.4f)", x_ardrone_, y_ardrone_, z_ardrone_);

            // only altitude
            takeoff::GroundtruthAltitude gt_altitude;
            gt_altitude.header.stamp = ros::Time::now();
            gt_altitude.header.seq = ros::Time::now().toSec();
            gt_altitude.header.frame_id = "odom"; // this should be a fixed world coordinate
            gt_altitude.gt_altitude = z_ardrone_;
            altitude_pub_.publish(gt_altitude);

            geometry_msgs::PoseStamped quadrotor_real_position;
            quadrotor_real_position.header.stamp = ros::Time::now();
            quadrotor_real_position.header.seq = ros::Time::now().toSec();
            quadrotor_real_position.header.frame_id = "/odom";
            quadrotor_real_position.pose.orientation.w = 1;
            quadrotor_real_position.pose.orientation.x = 0;
            quadrotor_real_position.pose.orientation.y = 0;
            quadrotor_real_position.pose.orientation.z = 0;
            quadrotor_real_position.pose.position.x = x_ardrone_;
            quadrotor_real_position.pose.position.y = y_ardrone_;
            quadrotor_real_position.pose.position.z = z_ardrone_;
            quadrotor_position_pub_.publish(quadrotor_real_position);

            // tf::Transform transform;
            // transform.setOrigin(tf::Vector3(x_ardrone_, y_ardrone_, z_ardrone_));
            // tf::Quaternion q;
            // tf::quaternionMsgToTF(gms_ardrone_.response.pose.orientation, q);
            // transform.setRotation(q);
            // br_.sendTransform(tf::StampedTransform(transform, gms_ardrone_.response.header.stamp,
            //                                        "world", "ardrone/base_link"));
        }


        // summit_xl
        if (!get_model_state_.call(gms_summit_)) {
            ROS_ERROR("Failed to call service summit");
        }
        if (!gms_summit_.response.success) {
            ROS_ERROR("Response to service ardrone failed");
        }
        else {
            x_summit_ = gms_summit_.response.pose.position.x;
            y_summit_ = gms_summit_.response.pose.position.y;
            z_summit_ = gms_summit_.response.pose.position.z;
            ROS_INFO("Summit position in world = (%.4f, %.4f, %4f) \n\n", x_summit_, y_summit_, z_summit_);

            geometry_msgs::PoseStamped summit_real_position;
            summit_real_position.header.stamp= ros::Time::now();
            summit_real_position.header.seq = ros::Time::now().toSec();
            summit_real_position.header.frame_id = "/odom";
            summit_real_position.pose.orientation.w=1;
            summit_real_position.pose.orientation.x=0;
            summit_real_position.pose.orientation.y=0;
            summit_real_position.pose.orientation.z=0;
            summit_real_position.pose.position.x=x_summit_;
            summit_real_position.pose.position.y=y_summit_;
            summit_real_position.pose.position.z=z_summit_;
            summit_position_pub_.publish(summit_real_position);

            // tf::Transform transform;
            // transform.setOrigin(tf::Vector3(x_summit_, y_summit_, 0.0));
            // tf::Quaternion q;
            // tf::quaternionMsgToTF(gms_summit_.response.pose.orientation, q);
            // transform.setRotation(q);
            // br_.sendTransform(tf::StampedTransform(transform, gms_summit_.response.header.stamp,
            //                                        "world", "summit_xl/base_footprint"));
        }

    } //end of while loop

} // end of RobotStateBroadcaster constructor

int main(int argc, char** argv) {

    ros::init(argc, argv, "robot_state_broadcaster");
    RobotStateBroadcaster robotStateBroadcaster;

    ///////////

    // ros::NodeHandle nh;

    // //std::string summit_frame = "/summit_xl/base_link";
    // std::string summit_frame = "/ardrone_base_bottomcam";
    // std::string quadrotor_frame = "/bottom_link";


    // // create the listener
    // tf::TransformListener listener;
    // listener.waitForTransform(quadrotor_frame, summit_frame, ros::Time(), ros::Duration(1.0));


    // ros::Rate rate(10);
    // while (nh.ok())
    // {
    //     tf::StampedTransform transform;
    //     try
    //     {
    //         listener.lookupTransform(quadrotor_frame, summit_frame, ros::Time(0), transform);

    //         // construct a pose message
    //         geometry_msgs::PoseStamped pose_stamped;
    //         pose_stamped.header.frame_id = quadrotor_frame;
    //         pose_stamped.header.stamp = ros::Time::now();

    //         pose_stamped.pose.orientation.x = transform.getRotation().getX();
    //         pose_stamped.pose.orientation.y = transform.getRotation().getY();
    //         pose_stamped.pose.orientation.z = transform.getRotation().getZ();
    //         pose_stamped.pose.orientation.w = transform.getRotation().getW();

    //         pose_stamped.pose.position.x = transform.getOrigin().getX();
    //         pose_stamped.pose.position.y = transform.getOrigin().getY();
    //         pose_stamped.pose.position.z = transform.getOrigin().getZ();

    //         ROS_INFO("%f, %f, %f", pose_stamped.pose.position.x,
    //                  pose_stamped.pose.position.y,
    //                  pose_stamped.pose.position.z);



    //         tf::Matrix3x3 m = transform.getBasis();
    //         double roll, pitch, yaw;
    //         m.getEulerYPR(yaw, pitch, roll);

    //         /*ROS_INFO("\n%f %f %f\n%f %f %f\n%f %f %f", m[0][0], m[0][1], m[0][2],
    //                  m[1][0], m[1][1], m[1][2],
    //                  m[2][0], m[2][1], m[2][2]);*/

    //         ROS_INFO("%f, %f, %f", roll * 180 / PI, pitch * 180 / PI, yaw * 180 / PI);

    //         m.getRPY(roll, pitch, yaw);
    //         ROS_INFO("%f, %f, %f\n\n", roll * 180 / PI, pitch * 180 / PI, yaw * 180 / PI);

    //         /*if (is_stamped)
    //             p_pub.publish(pose_stamped);
    //         else
    //             p_pub.publish(pose_stamped.pose);*/
    //     }
    //     catch (tf::TransformException &ex)
    //     {
    //         ROS_INFO("could not get transform");
    //         // just continue on
    //     }

    //     rate.sleep();
    // }
}