#include <ped_traj_pred/predict_filter.hpp>

using NEWMAT::Matrix;
using NEWMAT::ColumnVector;

PredictFilter::PredictFilter() {   //Constructor

    ROS_INFO("[PredictFilter] -- constructor ");

    ros::NodeHandle  nh("~");  // Create a ROS nodehandle

    filter_initialized = false;

    // topics and subscribers
    std::string visualization_topic_name;
    std::string future_pose_topic_name;
    std::string future_path_topic_name;

    nh.param("visualization_topic", visualization_topic_name, std::string("visualization_kalman"));
    nh.param("future_pose_topic", future_pose_topic_name, std::string("future_pos"));
    nh.param("future_path_topic", future_path_topic_name, std::string("kalman_pred_path"));

    visualization_pub = nh.advertise<visualization_msgs::Marker>(visualization_topic_name, 1);
    future_pos_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> (future_pose_topic_name, 1);
    future_path_pub = nh.advertise<ped_traj_pred::PathWithId>(future_path_topic_name, 1);

    // visualization
    points.ns = line_strip.ns = "future_position";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_strip.scale.x = 0.1;
    line_strip.scale.y = 0.1;

    points.color.r = 1.0;
    points.color.g = 1.0;
    points.color.a = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // path params
    double pub_freq;
    double path_time;

    nh.param("pub_freq", pub_freq, 1.0);
    nh.param("path_time", path_time, 6.0);
    num_pos = (int) (path_time / time_step);

    ROS_INFO("[PredictFilter] -- Advertise topics ");
    ROS_INFO("[PredictFilter] -- path_time: %f", path_time);
    ROS_INFO("[PredictFilter] -- time_step: %f", time_step);
    ROS_INFO("[PredictFilter] -- num_pos:   %d", num_pos);

    publish_timer = nh.createTimer(ros::Duration(1.0 / pub_freq), &PredictFilter::publishSpin, this);
    publish_timer.stop();

    ROS_INFO("[PredictFilter] -- Init oK");
}

PredictFilter::~PredictFilter() {}

void PredictFilter::publishSpin(const ros::TimerEvent& e) {

    ROS_INFO("os::time %f - Predicting...", ros::Time::now().toSec());

    // POSESTAMPED
    // geometry_msgs::PoseWithCovarianceStamped pos_with_cov;
    // pos_with_cov.header.frame_id = output_frame_id;
    // pos_with_cov.header.stamp = ros::Time::now();

    // PATH
    ped_traj_pred::PathWithId kal_path;
    kal_path.path.poses.clear();
    kal_path.path.poses.resize(num_pos);
    kal_path.path.header.frame_id = output_frame_id;
    kal_path.path.header.stamp = ros::Time::now();
    kal_path.path_id.data = path_id;

    // VIZ
    // points.header.stamp = line_strip.header.stamp = ros::Time();
    // points.header.frame_id = line_strip.header.frame_id = output_frame_id;
    // points.points.clear();
    // line_strip.points.clear();

    // We obtain the value of the path's first measurement like so:
    x_pred = x;
    P_pred = P;

    ros::Time estimation_stamp = ros::Time::now();

    // prediction
    for (unsigned int pos_counter = 0; pos_counter <= num_pos ; pos_counter++) {
        if (pos_counter < num_pos) {

            estimation_stamp += ros::Duration(time_step);
            predictPred();
            correctPred();

            // visualizations
            // mark.x = x_pred.element(0);
            // mark.y = x_pred.element(1);
            // mark.z = 0;
            // points.points.push_back(mark);
            // line_strip.points.push_back(mark);

            // visualization_pub.publish(points);
            // visualization_pub.publish(line_strip);

            // PoseStamped
            // pos_with_cov.header.stamp += ros::Duration(time_step);
            // pos_with_cov.pose.pose.position.x = x_pred.element(0);
            // pos_with_cov.pose.pose.position.y = x_pred.element(1);
            // future_pos_pub.publish(pos_with_cov); // publish future position

            // Path
            geometry_msgs::PoseStamped pose_at_x;
            pose_at_x.header.stamp = estimation_stamp;
            pose_at_x.header.frame_id = output_frame_id;
            pose_at_x.pose.orientation.w = 1;
            pose_at_x.pose.position.x = x_pred.element(0);
            pose_at_x.pose.position.y = x_pred.element(1);

            ROS_INFO("[%d] x_pred: %f - y_pred: %f", pos_counter, x_pred.element(0), x_pred.element(1));

            kal_path.path.poses[pos_counter] = pose_at_x;
        }
        else if (pos_counter == num_pos) {
            future_path_pub.publish(kal_path); // publish predicted traj with a path with id
            ROS_INFO("---");
        }
    }
}

void PredictFilter::refreshPos(float pos_x, float pos_y) {
    ColumnVector z = ColumnVector(2); // measurement

    z.element(0) = pos_x;
    z.element(1) = pos_y;

    if ( filter_initialized == true) {
        predict();
        update(z);
        correct();
    }
    else {
        initializeFilter(z);
        filter_initialized = true;
    }
}

double PredictFilter::getTimeStep() const {
    return time_step;
}

void PredictFilter::setTimeStep(double value) {
    time_step = value;
}

int PredictFilter::getPathId() const {
    return path_id;
}

void PredictFilter::setPathId(int value) {
    path_id = value;
}

