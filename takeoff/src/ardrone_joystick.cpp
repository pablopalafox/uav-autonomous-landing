/*
 * ardrone_joystick:
 *
 * This software provides a connection between a PS3-Joystick and the ardrone_brown - drone-driver
 *
 * It receives the joystick state from the joy-node and publishes the corresponding commands to the driver
 *
 * Author: Nikolas Engelhard
 * Modified: Pablo R. Palafox (2019)
 *
 *
 */


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

constexpr int PUBLISH_FREQ = 50;
constexpr double MAX_LINEAR_SPEED = 3.0;
constexpr double MAX_YAW_SPEED = 2.0;
constexpr double INCR_SPEED = 0.2;

using namespace std;

struct TeleopArDrone
{
    ros::Subscriber joy_sub;
    ros::Publisher pub_takeoff, pub_land, pub_force_land, pub_toggle_state, pub_vel;

    bool got_first_joy_msg;

    bool toggle_pressed_in_last_msg;
    bool cam_toggle_pressed_in_last_msg;
    std_srvs::Empty srv_empty;

    ros::NodeHandle nh_;
    geometry_msgs::Twist twist;
    ros::ServiceClient srv_cl_cam;

    double scale_linear;
    double scale_yaw;

    void joyCb(const sensor_msgs::JoyConstPtr joy_msg) {

        if (!got_first_joy_msg) {
            ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
            if (joy_msg->buttons.size() != 17 || joy_msg->axes.size() != 6) {
                ROS_FATAL("This joystick does not look like a PS4-Joystick");
                ROS_INFO("It has %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
            }
            got_first_joy_msg = true;
        }

        // mapping from joystick to velocity
        double scale = 1;

        // button 2 (triangle): augment linear scale
        bool speed_up = joy_msg->buttons.at(2);
        if (speed_up) {
            if (scale_linear < MAX_LINEAR_SPEED) {
                scale_linear += INCR_SPEED;
            }
        }

        // button 0 (cross): reduce linear scale
        bool speed_down = joy_msg->buttons.at(0);
        if (speed_down) {
            if (scale_linear > 1.0) {
                scale_linear -= INCR_SPEED;
            }
        }

        twist.linear.x = scale * scale_linear * joy_msg->axes[1]; // forward, backward
        twist.linear.y = scale * scale_linear * joy_msg->axes[0]; // left right
        twist.linear.z = scale * scale_linear * joy_msg->axes[4]; // up down
        twist.angular.z = scale * joy_msg->axes[3]; // yaw

        // button 4 (L1): dead man switch
        bool take_off_pressed = joy_msg->buttons.at(4);

        // button 5 (R1): dead man switch
        bool land_pressed = joy_msg->buttons.at(5);

        // button 7 (R2):
        bool force_land_pressed = joy_msg->buttons.at(7);

        // button 9 (start): switch emergency state
        bool emergency_toggle_pressed = joy_msg->buttons.at(9);

        // button 8 (select): switch camera mode
        bool cam_toggle_pressed = joy_msg->buttons.at(8);

        if (take_off_pressed) {
            ROS_INFO("L1 was pressed, Taking off!");
            pub_takeoff.publish(std_msgs::Empty());
        }

        if (land_pressed) {
            ROS_INFO("R1 was pressed, landing");
            pub_land.publish(std_msgs::Empty());
        }

        if (force_land_pressed) {
            ROS_INFO("R2 was pressed, forcing landing");
            pub_force_land.publish(std_msgs::Empty());
        }

        // toggle only once!
        if (!toggle_pressed_in_last_msg && emergency_toggle_pressed) {
            ROS_INFO("Changing emergency status");
            pub_toggle_state.publish(std_msgs::Empty());
        }
        toggle_pressed_in_last_msg = emergency_toggle_pressed;

        if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed) {
            ROS_INFO("Changing Camera");
            if (!srv_cl_cam.call(srv_empty)) {
                ROS_INFO("Failed to toggle Camera");
            }
        }
        cam_toggle_pressed_in_last_msg = cam_toggle_pressed;

    }

    TeleopArDrone() {
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;

        got_first_joy_msg = false;

        joy_sub = nh_.subscribe("/joy", 1, &TeleopArDrone::joyCb, this);
        toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

        pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
        pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1);
        pub_force_land    = nh_.advertise<std_msgs::Empty>("/ardrone/force_land", 1);
        pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 1);
        pub_vel           = nh_.advertise<geometry_msgs::Twist>("/ardrone/cmd_vel", 1);
        srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam", 1);

        scale_linear = 1.0;
        scale_yaw = 1.0;
    }

    void send_cmd_vel() {
        pub_vel.publish(twist);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_teleop");

    ROS_INFO("Started ArDrone joystick-Teleop");
    ROS_INFO("Press L1 to toggle emergency-state");
    ROS_INFO("Press and hold L2 for takeoff");
    ROS_INFO("Press 'select' to choose camera");

    TeleopArDrone teleop;
    ros::Rate pub_rate(PUBLISH_FREQ);

    while (teleop.nh_.ok())
    {
        ros::spinOnce();
        teleop.send_cmd_vel();
        pub_rate.sleep();
    }

    return 0;
}