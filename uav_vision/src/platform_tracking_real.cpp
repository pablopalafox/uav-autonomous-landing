#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <hector_uav_msgs/Altimeter.h>
#include <string>
#include <std_msgs/Int8.h>
//#include <ardrone_autonomy/Navdata.h>



//se debera hacer el consttrol con una frecuencia de menor que 1/(1/20)* frecuencia de recepcion de posicion del coche aprox
//Estimacion de la posicion se emite a 13Hz -> 0.03s - 0.2s => 50*13 = 650Hz , 0.0006s - 0.04s

double sumatorio_error_x = 0;
double sumatorio_error_y = 0;

double derror_x = 0;
double derror_y = 0;

double position_x0;
double position_y0;

float t_ref = 0;

int init_takeoff=0;
int init_landing=0;

class Platform_Tracking
{
  public:
    Platform_Tracking();

    geometry_msgs::Twist vel;
    std_msgs::Empty emptyMsg;
    std_srvs::Empty emptySrv;

    double current_vel;
    double position_x;
    double position_y;
    double position_z;
  
    ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
    ros::Publisher vel_pub_ardrone_;
    // ros::Publisher takeoff_pub ;
    // ros::Publisher land_pub;
    // ros::Publisher reset_pub;
    // ros::Subscriber joy_sub_;//! It will be suscribed to the joystick
    ros::Subscriber altitude_subs_;
    
    ros::ServiceClient client_1;//!togglecam
    ros::ServiceClient client_2;//!flattrim
    
    ros::Subscriber platform_position_;
    
    ros::Publisher position_quad_base_;
    // ros::Publisher estimated_platform_position_;

    ros::Subscriber takeoff_subscriber_, landing_subscriber_;

    

    //Transformations
    tf::TransformListener tf_listener;
    tf::StampedTransform transform, transform_world;

    // void  flattrim(float duration);
    // void  land(float duration);
    // void  takeoff(float duration);
    // void  giro(float duration, char  sentido);
    // void  desplazamiento_x(float duration, char sentido);
    // void  desplazamiento_y(float duration, char sentido);
    void tracking(const geometry_msgs::PoseStamped& centroid);
    void altitude_control(const ardrone_autonomy::NavdataConstPtr& ardrone_navdata);
    void takeoff(const std_msgs::Int8 &takeoff_signal);
    void landing(const std_msgs::Int8 &landing_signal);

  private:
    ros::NodeHandle nh_;
 };


Platform_Tracking::Platform_Tracking()
{
	std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity
    std::string platform_position_topic_;
    std::string position_quad_base_topic_;
    // std::string estimated_platform_position_topic_;
    std::string ardrone_navdata_;

	current_vel = 1.0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    nh_.param("cmd_topic_vel", cmd_topic_vel_, std::string("/cmd_vel"));
    nh_.param("platform_position_topic", platform_position_topic_, std::string("/platform/estimated_position"));
    nh_.param("position_quad_base_topic", position_quad_base_topic_, std::string("/quadrotor/tracking/target_position"));
    // nh_.param("estimated_platform_position_topic", estimated_platform_position_topic_, std::string("world_estimated_position"));
    
    nh_.param("ardrone_navdata", ardrone_navdata_, std::string("/ardrone/navdata"));

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());

  	// Publish through the node handle Twist type messages to the ARDrone_ctrl/command topic
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
    vel_pub_ardrone_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//Publish to topics
	
    //takeoff_pub = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	// land_pub = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
	// reset_pub = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);
    position_quad_base_ = nh_.advertise<geometry_msgs::PoseStamped>(position_quad_base_topic_, 1);
    //estimated_platform_position_ = nh_.advertise<geometry_msgs::PoseStamped>(estimated_platform_position_topic, 1);
    

	//Ask for services
    /*
	client_1 = nh_.serviceClient<std_srvs::Empty>("ardrone/togglecam");
	client_2 = nh_.serviceClient<std_srvs::Empty>("ardrone/flattrim");
    */

	//Subscribe
	platform_position_ = nh_.subscribe(platform_position_topic_, 1, &Platform_Tracking::tracking, this);
    altitude_subs_ = nh_.subscribe(ardrone_navdata_, 1, &Platform_Tracking::altitude_control, this);

    takeoff_subscriber_ = nh_.subscribe("/quadrotor/takeoff_topic", 1, &Platform_Tracking::takeoff, this);
    landing_subscriber_ = nh_.subscribe("/quadrotor/landing_topic", 1, &Platform_Tracking::landing, this);



    int i = 0;
    int j = 0;

    tf_listener.waitForTransform("/ardrone_base_link","/ardrone_base_bottomcam", ros::Time(0), ros::Duration(3.0));         
    try{
        tf_listener.lookupTransform("/base_link","/ardrone_base_bottomcam",ros::Time(0), transform);
        ROS_INFO("Trying to lookupTransform_1");
        i=3;
        t_ref = ros::Time::now().toSec();
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    while(i < 2)
    {
        ROS_ERROR("%d",i);
        tf_listener.waitForTransform("/ardrone_base_link","/ardrone_base_bottomcam", ros::Time(0), ros::Duration(3.0));
        try{
            tf_listener.lookupTransform("/ardrone_base_link","/ardrone_base_bottomcam",ros::Time(0), transform);
            ROS_INFO("Trying to lookupTransform_2");
            i=3;
            t_ref = ros::Time::now().toSec();
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            i++;
        }
    }
} // end del constructor


void Platform_Tracking::takeoff(const std_msgs::Int8& takeoff_signal){
   
   init_takeoff = (int) takeoff_signal.data;
   ROS_INFO("init_takeoff: %d", init_takeoff);
   ROS_INFO("init_landing: %d", init_landing);

}


void Platform_Tracking::landing(const std_msgs::Int8& landing_signal){
    
    init_landing = (int) landing_signal.data;
    ROS_INFO("init_takeoff: %d", init_takeoff);
    ROS_INFO("init_landing: %d", init_landing);

}


void Platform_Tracking::altitude_control(const ardrone_autonomy::NavdataConstPtr& ardrone_navdata){
    
    position_z = ardrone_navdata->altd;
    ROS_INFO_STREAM("La altura del ardrone es de " << position_z);
    
    //takeoff control:
    if (init_takeoff)
    {
        vel.linear.z += 0.05;
        if (position_z > 11.0)
        {
            vel.linear.z = 0.0;
            init_takeoff = 0;
        }
        vel_pub_.publish(vel);
        vel_pub_ardrone_.publish(vel);
    }



}


void  Platform_Tracking::tracking(const geometry_msgs::PoseStamped &centroid){

    if (!init_takeoff)
    {
        ROS_INFO("LOCALIZADO");
        ROS_INFO("init_takeoff: %d", init_takeoff);
        ROS_INFO("init_landing: %d", init_landing);
        tf::Vector3  pose_base_link;
        //tf::Vector3  pose_in_world;
        tf::Vector3  point( centroid.pose.position.x, centroid.pose.position.y, centroid.pose.position.z );

        //Recalcular transformaciones del frame dinámico /image_plane respecto del frame estático /map
        // tf_listener.waitForTransform("/world","/downward_cam_optical_frame", ros::Time(0), ros::Duration(0.5)); //0.05
        // tf_listener.lookupTransform("/world","/downward_cam_optical_frame",ros::Time(0), transform_world);

        pose_base_link = transform * point;
        // pose_in_world = transform_world * point;

        ROS_INFO("El punto (%f,  %f , %f)  en metros respecto a la camara", point.getX(), point.getY(), point.getZ());
        ROS_INFO("platformXL Position: (%f,  %f , %f)  en metros respecto del quadrotor", pose_base_link.getX(), pose_base_link.getY(), pose_base_link.getZ());
        // ROS_INFO("platformXL Position: (%f,  %f , %f)  en metros respecto del mundo", pose_in_world.getX(), pose_in_world.getY(), pose_in_world.getZ());

        ROS_INFO("Tiempo desde comienzo = %.5f", ros::Time::now().toSec() - t_ref);

        // REGULADOR P - proporcional al error ///////////////////////////////////////
        //vel.linear.x = 0.57*pose_base_link.getX();//calculo de K_c
        //vel.linear.x = -0.03;//default -vx of the quadrotor //
        //vel.linear.y = 0.56*pose_base_link.getY();

        /*vel.linear.x = 0.3*current_vel*pose_base_link.getX();
        vel.linear.y = 0.3*current_vel*pose_base_link.getY();*/
        //vel.linear.x = transform.getOrigin().x(),transform.getOrigin().x());
        //////////////////////////////////////////////////////////////////////////////

        //REGULADOR PI - proporcional a la integral del error ////////////////////////
        sumatorio_error_x += pose_base_link.getX(); //Sistema simétrico
        sumatorio_error_y += pose_base_link.getY();

        //vel.linear.x = 0.25*current_vel*pose_base_link.getX() + 0.02*current_vel*sumatorio_error_x ;
        //vel.linear.y = 0.25*current_vel*pose_base_link.getY() + 0.02*current_vel*sumatorio_error_x ;
        //////////////////////////////////////////////////////////////////////////////

        //REGULADOR PID - añadimos proporcionalidad ante la derivada del error ///////
        derror_x = pose_base_link.getX() - position_x0;
        derror_y = pose_base_link.getY() - position_y0;

        //BUENO BUENO BUENO
        
        vel.linear.x = 0.1045*pose_base_link.getX() + 0.0011*sumatorio_error_x + 0.008*derror_x;//bien
        vel.linear.y = 0.1045*pose_base_link.getY() + 0.0011*sumatorio_error_y + 0.008*derror_y;
        
        //el que podría valer para el REAL
        //vel.linear.x = 0.35*pose_base_link.getX() + 0.0007*sumatorio_error_x + 0.005*derror_x;//bien
        //vel.linear.y = 0.35*pose_base_link.getY() + 0.0007*sumatorio_error_y + 0.005*derror_y;


        //vel.linear.x = 0.35*current_vel*pose_base_link.getX() + 0.003*current_vel*sumatorio_error_x + 0.025*current_vel*derror_x;
        //vel.linear.y = 0.35*current_vel*pose_base_link.getY() + 0.003*current_vel*sumatorio_error_x + 0.025*current_vel*derror_y;

        //Estimación de Ziegler-Nichols
        //vel.linear.x = 0.342*current_vel*pose_base_link.getX() + 0.342*current_vel*sumatorio_error_x + 0.0427*current_vel*derror_x;
        //vel.linear.y = 0.336*current_vel*pose_base_link.getY() + 0.388*current_vel*sumatorio_error_x + 0.0726*current_vel*derror_y;

        position_x0 = pose_base_link.getX();
        position_y0 = pose_base_link.getY();

        //////////////////////////////////////////////////////////////////////////////
        //////////////////////// LANDING CONTROL /////////////////////////////////////
        
        if (init_landing)
        {
            if ( (position_z < 60) && (position_z > 3) ) // in meters
            {
                vel.linear.z = -0.7;
            }
            else if (position_z <= 3)
            {
                vel.linear.z = -0.25;

            }
        }

        //////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////

        geometry_msgs::PoseStamped real_centroid; //posicion del centroide respecto del quadrotor base_link
            real_centroid.header.stamp= ros::Time::now();
            real_centroid.header.seq = ros::Time::now().toSec() - t_ref;
            real_centroid.header.frame_id = "/ardrone_base_link";
            real_centroid.pose.orientation.w=1;
            real_centroid.pose.orientation.x=0;
            real_centroid.pose.orientation.y=0;
            real_centroid.pose.orientation.z=0;
            real_centroid.pose.position.x=pose_base_link.getX();
            real_centroid.pose.position.y=pose_base_link.getY();
            real_centroid.pose.position.z=pose_base_link.getZ();


        // geometry_msgs::PoseStamped estimated_centroid_in_world; //posicin del centride de la etimacion en el mundo
        //     estimated_centroid_in_world.header.stamp= ros::Time::now();
        //     estimated_centroid_in_world.header.seq = ros::Time::now().toSec() - t_ref;
        //     estimated_centroid_in_world.header.frame_id = "/world";
        //     estimated_centroid_in_world.pose.orientation.w=1;
        //     estimated_centroid_in_world.pose.orientation.x=0;
        //     estimated_centroid_in_world.pose.orientation.y=0;
        //     estimated_centroid_in_world.pose.orientation.z=0;
        //     estimated_centroid_in_world.pose.position.x=pose_in_world.getX();
        //     estimated_centroid_in_world.pose.position.y=pose_in_world.getY();
        //     estimated_centroid_in_world.pose.position.z=pose_in_world.getZ();

        ROS_INFO("Sending Velocity:  (%f,  %f, %f)",vel.linear.x, vel.linear.y, vel.linear.z);
        ROS_INFO("Acumulated errors:  (%f,  %f)\n\n",sumatorio_error_x, sumatorio_error_y);

        position_quad_base_.publish(real_centroid);
        //estimated_platform_position_.publish(estimated_centroid_in_world);
        
        vel_pub_.publish(vel);
        vel_pub_ardrone_.publish(vel);

    }//end del if enorme
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "platform_tracking_real");
    Platform_Tracking platform_tracking;
    ros::Duration(2).sleep();
    ROS_INFO("About to start sequence");
    ros::spin();
}
