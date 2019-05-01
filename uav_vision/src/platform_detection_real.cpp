#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <ardrone_autonomy/Navdata.h>
#include <std_srvs/Empty.h>




#include <tf/transform_listener.h>

#define DEFAULT_SIZE 0.64

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

static const char WINDOW[] = "Image Processed";
static const char WINDOW_NAME_BLUR[] = "window_image_blur";
static const char WINDOW_NAME_FINAL[] = "window_image_final";

cv::Mat img_mask;
cv::Mat img_hsv;

cv::Mat image;




int H_min = 0;
int H_max = 179;
int S_min = 0;
int S_max = 196;
int V_min = 0;
int V_max = 170;

int images_read = 0;
int times_detected = 0;
int false_positives = 0;
int obj_detected = 0;

// image Subscriber and Publisher
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

class Platform_Detection
{
  
    // ROS PUBLISHERS
    ros::Publisher platform_position_;

    // ROS SUBSCRIBERS
    ros::Subscriber altitude_;
    ros::Subscriber cam_info_;
    ros::Subscriber imu_info_;

    // ROS SERVICE
    std_srvs::Empty emptySrv;

    // ROS CLIENTS
    ros::ServiceClient client_1;//!togglecam
    ros::ServiceClient client_2;//!flattrim


    double fx;
    double fy;
    double cx;
    double cy;
    double T;
    
    double altitude_quadrotor;
    
    geometry_msgs::Twist vel;
    
    double angle_x;
    double angular_velocity_x;
    
    double angle_y;
    double angular_velocity_y;
    
    double linear_acceleration_x;
    double linear_acceleration_y;

    double teoric_size;

    double altura_platform;

    //VARIABLES FOR DRAWING THE LANDING_PLATFORM SHAPE
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> approx;

public:
    Platform_Detection();
    void height(const ardrone_autonomy::NavdataConstPtr& ardrone_navdata);
    void cam_info(const sensor_msgs::CameraInfo& cam_parameters);
    void angle(const sensor_msgs::ImuConstPtr& hector_imu);
    void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image);

    
    
}; //end of class platform_Detection


Platform_Detection::Platform_Detection()
{
    ROS_INFO("Init Class: platform_Detection");
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string platform_position_topic_;
    std::string ardrone_navdata_topic_;
    std::string imu_topic_;

    
    

    // GETTING ROS PARAMETERS
    nh_.param("image_topic", image_topic_, std::string("/ardrone/image_raw"));
    nh_.param("cam_info_topic", camera_info_topic_, std::string("/ardrone/camera_info"));
    nh_.param("platform_position_topic", platform_position_topic_, std::string("/platform/estimated_position"));
    nh_.param("ardrone_navdata_topic", ardrone_navdata_topic_, std::string("/ardrone/navdata"));
    nh_.param("imu_topic", imu_topic_, std::string("/ardrone/imu"));

    nh_.param("H_min", H_min, 0);
    nh_.param("H_max", H_max, 179);
    nh_.param("S_min", S_min, 0);
    nh_.param("S_max", S_max, 156);
    nh_.param("V_min", V_min, 0);
    nh_.param("V_max", V_max, 164);

    nh_.param("teoric_size", teoric_size, DEFAULT_SIZE);
    nh_.param("altura_platform", altura_platform, 0.392);



    //TOPICS

    //Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_topic_, 1, &Platform_Detection::colorDetectionCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //We subscribe to the ALTITUDE_TOPIC of the quadrotor
    altitude_ = nh_.subscribe(ardrone_navdata_topic_, 1, &Platform_Detection::height, this);
    
    //We subscribe to the IMU_TOPIC of the quadrotor
    imu_info_ = nh_.subscribe(imu_topic_, 1, &Platform_Detection::angle, this);

    //We subscribe to the CAMERA_INFO_TOPIC of the quadrotor
    cam_info_ = nh_.subscribe(camera_info_topic_, 1, &Platform_Detection::cam_info, this);

    //We publish on the platform_POSITION_TOPIC 
    platform_position_ = nh_.advertise<geometry_msgs::PoseStamped>(platform_position_topic_, 1);


    //SERVICES
    //Ask for services
    client_1 = nh_.serviceClient<std_srvs::Empty>("ardrone/togglecam");
    client_2 = nh_.serviceClient<std_srvs::Empty>("ardrone/flattrim");

    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    bool success = client_1.call(req,resp);
    if (success) ROS_INFO_STREAM("Camera toggled");
    else ROS_INFO_STREAM("Error while trying to toggle camera");



    //TRACKBAR
    cv::namedWindow("Ballplatform");
    cv::createTrackbar("H_min","Ball",&H_min,179,NULL);
    cv::createTrackbar("H_max","Ball",&H_max,179,NULL);
    cv::createTrackbar("S_min","Ball",&S_min,255,NULL);
    cv::createTrackbar("S_max","Ball",&S_max,255,NULL);
    cv::createTrackbar("V_min","Ball",&V_min,255,NULL);
    cv::createTrackbar("V_max","Ball",&V_max,255,NULL);



    ROS_INFO("Setup completed");




}
   
void Platform_Detection::height(const ardrone_autonomy::NavdataConstPtr& ardrone_navdata)
{
    altitude_quadrotor = ardrone_navdata->altd;

    
}


void Platform_Detection::angle(const sensor_msgs::ImuConstPtr& ardrone_imu)
{

  angle_x               = ardrone_imu->orientation.x;
  angular_velocity_x    = ardrone_imu->angular_velocity.x;
 
  angle_y               = ardrone_imu->orientation.y;
  angular_velocity_y    = ardrone_imu->angular_velocity.y;
 
  linear_acceleration_x = ardrone_imu->linear_acceleration.x;
  linear_acceleration_y = ardrone_imu->linear_acceleration.y;

}


void Platform_Detection::cam_info(const sensor_msgs::CameraInfo& cam_parameters){
    //ROS_INFO("Parameters from the pinhole model");
    fx = cam_parameters.K.at(0);
    fy = cam_parameters.K.at(4);
    cx = cam_parameters.K.at(2);
    cy = cam_parameters.K.at(5);
    T  = cam_parameters.K.at(1);
    //ROS_INFO("Los parametros son  (fx,fy)=(%f , %f)", fx, fy);
}


void Platform_Detection::colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
        image = cv_ptr->image;
        images_read++;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert image: %s", e.what());
        return;
    }

    cv::Mat img_mask, img_hsv; 
    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(H_min,S_min,V_min),cv::Scalar(H_max,S_max,V_max),img_mask); 
    //cv::imshow(WINDOW, img_mask);
    cv::waitKey(3);

    //We negate the image
    cv::Mat img_negated;
    cv::bitwise_not ( img_mask, img_negated );
   
   
    //Find countours on the image
    cv::findContours(img_negated, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    for (int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)* 0.02, true);

        cv::Point pt;
        for(int j=0;j<approx.size();j++)
        {
            //cv::Point point = approx[i];
            //ROS_INFO("extracting points X %d Y %d",point.x, point.y);

            if (j == approx.size()-1)
            {
                cv::line(image, approx.at(j), approx.at(0), cv::Scalar(0,255,0), 4);
            }
            else
            {
                cv::line(image, approx.at(j), approx.at(j+1), cv::Scalar(0,255,0), 4);
            }
        }

        // Centroid extraction----------------------------------------------------------------
        /// 1º Get the moments
        cv::Moments m;
        m = cv::moments( approx, false );

        /// 2º Get the mass centers: (centroid)
        cv::Point2f mc;
        mc = cv::Point2f( m.m10/m.m00 , m.m01/m.m00 );
        cv::line(image, mc, mc, cv::Scalar(0,255,255), 8); // we draw a yellow point for that mc

        /// PINHOLE inverse transformation ==> (x,y,z)= f(u,v) without distortion, (u,v)=mc ---------------------
        ROS_INFO("Centroid location");
        geometry_msgs::PoseStamped centroid;
            centroid.header.stamp= ros::Time::now();
            centroid.header.seq = ros::Time::now().toSec();
            centroid.header.frame_id = "/image_plane";
            centroid.pose.orientation.w=1;
            centroid.pose.orientation.x=0;
            centroid.pose.orientation.y=0;
            centroid.pose.orientation.z=0;
            
            //Centroide corregido
            centroid.pose.position.x=((mc.x - cx)*altitude_quadrotor - (T*(mc.y - cy)*altitude_quadrotor)/fy)/fx;
            centroid.pose.position.y=((mc.y - cy)*altitude_quadrotor)/fy;            
            centroid.pose.position.z=altitude_quadrotor - altura_platform;          
        
        ROS_INFO("Centro de masas en pixels del objetivo: (%f , %f)", mc.x, mc.y);
        ROS_INFO("La posicion del objetivo es: (%f , %f, %f) en metros respecto a la camara", centroid.pose.position.x, centroid.pose.position.y, centroid.pose.position.z);

        platform_position_.publish(centroid);

    }


    /// Show in a window
    namedWindow( "platform_DETECTED", CV_WINDOW_AUTOSIZE );
    imshow( "platform_DETECTED", image );


    


  

    image_pub_.publish(cv_ptr->toImageMsg());

    cv::waitKey(3);

                                         
}
   

int main(int argc, char** argv)
{
    ros::init(argc, argv, "platform_detection_real");
    
    Platform_Detection platform_detection;
    ros::spin();
    ROS_INFO("platform_detection::main.cpp::No error.");
    cv::destroyWindow(WINDOW);
    return 0;
}