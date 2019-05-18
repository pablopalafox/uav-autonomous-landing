/*************************************************************************
This file is licensed under a GPLv3 License.
GPLv3 License
Copyright (C) 2016-2019 Pablo R. Palafox (pablo.rodriguez-palafox@tum.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*************************************************************************/

#include <csignal>
#include <random>

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Empty.h>

#include <geometry_msgs/Twist.h>

constexpr double PI = 3.14159265359;
constexpr double NINETY_DEGREES = PI / 2.0;
constexpr double ROTATION_SHIFT = PI / 16.0;
constexpr int PUBLISH_FREQ = 50;

////////////////////////////////////////////////////////////////////////////////

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;


void mySigintHandler(int sig) {
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

////////////////////////////////////////////////////////////////////////////////

class SummitMoves {
public:
    ros::NodeHandle nh_;

    bool start = false;

    ros::Subscriber takeoff_sub_;
    ros::Publisher vel_pub;
    geometry_msgs::Twist twist;

    std::string trajectory;
    bool advancing = true;

    double max_linear_speed_x, max_yaw;

    double side_square;
    double radius;

    std::uniform_real_distribution<double> unif_interval;
    std::uniform_real_distribution<double> unif_linear_x;
    std::uniform_real_distribution<double> unif_yaw;
    std::default_random_engine re;

    SummitMoves();
    double doTrajectory();

    void takeoffCallback(const std_msgs::EmptyConstPtr &takeoff_signal);

    void stop(double interval);
    void advance(double interval);
    void reverse(double interval);
    double yawLeft90();

    double doBackAndForth();
    double doSquare();
    double doCircular();
    double doRandom();

};

SummitMoves::SummitMoves() {

    ros::NodeHandle nh_private_("~");

    takeoff_sub_ = nh_.subscribe("/ardrone/takeoff", 1, &SummitMoves::takeoffCallback, this);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/summit_xl_control/cmd_vel", 10);

    nh_private_.getParam("max_linear_speed_x", max_linear_speed_x);
    nh_private_.getParam("max_yaw", max_yaw);

    nh_private_.getParam("trajectory", trajectory);

    nh_private_.getParam("side_square", side_square);
    nh_private_.getParam("radius", radius);

    unif_interval = std::uniform_real_distribution<double>(0.0, 10.0);
    unif_linear_x = std::uniform_real_distribution<double>(-max_linear_speed_x, max_linear_speed_x);
    unif_yaw      = std::uniform_real_distribution<double>(-max_yaw, max_yaw);
}

void SummitMoves::takeoffCallback(const std_msgs::EmptyConstPtr & takeoff_signal) {
    ROS_INFO("Starting!");
    start = true;
}

double SummitMoves::doTrajectory() {
    if (trajectory == "backandforth") {
        ROS_INFO("Trajectory: %s", trajectory.c_str());
        return doBackAndForth();
    }
    else if (trajectory == "square") {
        ROS_INFO("Trajectory: %s", trajectory.c_str());
        return doSquare();
    }
    else if (trajectory == "circular") {
        ROS_INFO("Trajectory: %s", trajectory.c_str());
        return doCircular();
    }
    else if (trajectory == "random") {
        ROS_INFO("Trajectory: %s", trajectory.c_str());
        return doRandom();
    }
    else {
        ROS_FATAL("Trajectory %s is not defined", trajectory.c_str());
        std::raise(SIGINT);
    }
}

void SummitMoves::stop(double interval) {
    ROS_INFO("Stopping summit...");
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    vel_pub.publish(twist);
    ros::Duration(interval).sleep();
}

void SummitMoves::advance(double interval) {
    ROS_INFO("Advancing for %f sec", interval);

    twist.linear.x = max_linear_speed_x;
    twist.angular.z = 0.0;
    ROS_INFO("linear x: %f - yaw: %f", twist.linear.x, twist.angular.z);
    vel_pub.publish(twist);
}

void SummitMoves::reverse(double interval) {
    ROS_INFO("Reversing for %f sec", interval);

    twist.linear.x = - max_linear_speed_x;
    twist.angular.z = 0.0;
    ROS_INFO("linear x: %f - yaw: %f", twist.linear.x, twist.angular.z);
    vel_pub.publish(twist);
}

double SummitMoves::yawLeft90() {
    ROS_INFO("Yawing left for %f degrees", NINETY_DEGREES * 180 / PI);
    twist.linear.x = 0.0;
    twist.angular.z = 0.5;
    ROS_INFO("linear x: %f - yaw: %f", twist.linear.x, twist.angular.z);
    vel_pub.publish(twist);
    return 3.0; // so that we make a 90 degrees rotation
}

double SummitMoves::doBackAndForth() {
    double interval;
    if (advancing) {
        interval = side_square / max_linear_speed_x;
        stop(0.2);
        advance(interval);
    }
    else {
        interval = side_square / max_linear_speed_x;
        interval;
        stop(0.2);
        reverse(interval);
    }

    advancing = !advancing;
    return interval;
}

double SummitMoves::doSquare() {
    double interval = side_square / max_linear_speed_x;
    if (advancing) {
        stop(0.2);
        advance(interval);
    }
    else {
        stop(0.2);
        interval = yawLeft90();
    }

    advancing = !advancing;
    return interval;
}

double SummitMoves::doCircular() {
    twist.linear.x = max_linear_speed_x;
    // v = w * r --> for circular trajectory
    twist.angular.z = twist.linear.x / radius;
    ROS_INFO("linear x: %f - yaw: %f", twist.linear.x, twist.angular.z);
    vel_pub.publish(twist);
    return 5.0;
}

double SummitMoves::doRandom() {
    // define a uniform distribution from which to sample an interval
    double interval = unif_interval(re);
    twist.linear.x = unif_linear_x(re);
    twist.angular.z = unif_yaw(re);
    ROS_INFO("interval: %f", interval);
    ROS_INFO("linear x: %f - yaw: %f", twist.linear.x, twist.angular.z);
    vel_pub.publish(twist);
    return interval;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "summit_moves", ros::init_options::NoSigintHandler);
    ROS_INFO("Summit Moves");
    SummitMoves summit_moves;

    std::signal(SIGINT, mySigintHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    double sleep_for_seconds = 0.5;

    while (!summit_moves.start) {
        ROS_INFO("Waiting for start signal...");
        usleep(sleep_for_seconds * 1000000); // in microseconds
        ros::spinOnce();
    }

    ROS_INFO("Let's move...");

    while (!g_request_shutdown) {
        usleep(sleep_for_seconds * 1000000); // in microseconds
        sleep_for_seconds = summit_moves.doTrajectory();
    }

    summit_moves.stop(2.0);

    ros::shutdown();

    return 0;
}


