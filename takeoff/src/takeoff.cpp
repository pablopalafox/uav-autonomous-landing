#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include<stdio.h>
#include <termios.h>
#include <unistd.h>


const char key_8 = 56;
const char key_2 = 50;
const char key_4 = 52;
const char key_5 = 53;
const char key_6 = 54;
const char key_7 = 55;
const char key_0 = 48;
const char key_1 = 49;
const char key_3 = 51;
const char key_takeoff = 't';
const char key_land = 'l';



int getch()
{
    static struct termios oldt, newt;

    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
    return c;
}

class Takeoff {
private:
    ros::NodeHandle nh_;

    geometry_msgs::Twist quadrotor_velocity_;
    bool is_flying_;
    bool going_up_;
    bool reached_target_height_;

    //ros::Subscriber position_subscriber_;
    ros::Publisher velocity_publisher_;
    ros::Publisher pub_takeoff_, pub_land_;


public:
    Takeoff();
    //void getQuadrotorPosition(const geometry_msgs::PoseStamped& quadrotor_position_);
    void forwards();
    void backwards();
    void move_right();
    void move_left();
    void rotate_right();
    void rotate_left();
    void upwards();
    void downwards();
    void stop();
    void takeoff();
    void land();
};


Takeoff::Takeoff() {
    is_flying_ = false;
    going_up_ = false;
    reached_target_height_ = false;
    quadrotor_velocity_.linear.x = 0.0;
    quadrotor_velocity_.linear.y = 0.0;
    quadrotor_velocity_.linear.z = 0.0;

    //position_subscriber_ = nh_.subscribe("/position_quadrotor_modelstate", 1, &Takeoff::getQuadrotorPosition, this);
    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("/ardrone/cmd_vel", 1);
    pub_takeoff_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_land_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1);
}


/*void Takeoff::getQuadrotorPosition(const geometry_msgs::PoseStamped& quadrotor_position) {
    ROS_INFO("ASDFKJASD");
    if (!is_flying_) {
        pub_takeoff_.publish(std_msgs::Empty());
        is_flying_ = true;
    } else {
        if (quadrotor_position.pose.position.z < 5.5 && !going_up_) {
            ROS_INFO("Quadrotor z coordinate: %.2f", quadrotor_position.pose.position.z);
            ROS_INFO("Sending upward velocity");
            quadrotor_velocity_.linear.z = 1;
            going_up_ = true;
            //publish quadrotor velocity
            velocity_publisher_.publish(quadrotor_velocity_);
        }
        else if (quadrotor_position.pose.position.z >= 5.5 && !reached_target_height_) {
            ROS_INFO("Quadrotor z coordinate: %.2f", quadrotor_position.pose.position.z);
            ROS_INFO("Sending zero velocity");
            quadrotor_velocity_.linear.z = 0;
            reached_target_height_ = true;
            //publish quadrotor velocity
            velocity_publisher_.publish(quadrotor_velocity_);
        }


    }
}*/

void Takeoff::takeoff() {
    ROS_INFO("Takeoff...");
    pub_takeoff_.publish(std_msgs::Empty());
}

void Takeoff::land() {
    ROS_INFO("Land...");
    pub_land_.publish(std_msgs::Empty());
}

void Takeoff::forwards() {
    ROS_INFO("Forward...");
    quadrotor_velocity_.linear.x = 1;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::backwards() {
    ROS_INFO("Backwards...");
    quadrotor_velocity_.linear.x = -1;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::move_right() {
    ROS_INFO("Moving Right...");
    quadrotor_velocity_.linear.y = -1.0;
    velocity_publisher_.publish(quadrotor_velocity_);

}

void Takeoff::move_left() {
    ROS_INFO("Moving Left...");
    quadrotor_velocity_.linear.y = 1.0;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::rotate_right() {
    ROS_INFO("Rotating Right...");
    quadrotor_velocity_.angular.z = -1.0;
    velocity_publisher_.publish(quadrotor_velocity_);

}

void Takeoff::rotate_left() {
    ROS_INFO("Rotating Left...");
    quadrotor_velocity_.angular.z = 1.0;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::upwards() {
    ROS_INFO("Upwards...");
    quadrotor_velocity_.linear.z = 1;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::downwards() {
    ROS_INFO("Downwards...");
    quadrotor_velocity_.linear.z = -1;
    velocity_publisher_.publish(quadrotor_velocity_);
}

void Takeoff::stop() {
    ROS_INFO("Stopping...");
    quadrotor_velocity_.linear.x = 0;
    quadrotor_velocity_.linear.y = 0;
    quadrotor_velocity_.linear.z = 0;
    quadrotor_velocity_.angular.z = 0;
    velocity_publisher_.publish(quadrotor_velocity_);
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "takeoff");
    Takeoff takeoff;

    ros::Duration(2).sleep();
    ROS_INFO("About to start sequence");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ROS_INFO("HI");
        int c = getch();   // call your non-blocking input function
        ROS_INFO("\nPressed %d", c);

        switch (c) {
        case key_takeoff:
            takeoff.takeoff();
            break;
        case key_land:
            takeoff.land();
            break;
        case key_8:
            takeoff.forwards(); 
            break;
        case key_2:
            takeoff.backwards();
            break;
        case key_4:
            takeoff.move_left();
            break;
        case key_6:
            takeoff.move_right();
            break;
        case key_1:
            takeoff.rotate_left();
            break;
        case key_3:
            takeoff.rotate_right();
            break;
        case key_0:
            takeoff.downwards();
            break;
        case key_7:
            takeoff.upwards();
            break;
        case key_5:
            takeoff.stop();
            break;
        case 'q':
            break;
        }

        if (c == 'q') {
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // takeoff.land();
    ROS_INFO("Bye!");

}
