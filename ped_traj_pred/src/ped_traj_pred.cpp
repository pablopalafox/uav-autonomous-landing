#include <ped_traj_pred/ped_traj_pred.hpp>

PedTrajPred::PedTrajPred() {

    ros::NodeHandle  nh("~");

    // Store topic names
    std::string pose_topic_name; // This is the one that matters to us, regarding the UAV-UGV Coordination
    std::string pose_with_cov_topic_name;
    std::string odometry_pose_topic_name;

    int path_id_param;
    nh.param("path_id", path_id_param, 1);

    nh.param("pose_topic", pose_topic_name, std::string("/pose_topic"));
    nh.param("pose_with_cov_topic", pose_with_cov_topic_name, std::string("/pose_with_cov"));
    nh.param("odometry_pose_topic", odometry_pose_topic_name, std::string("/odom"));

    pose_sub = nh.subscribe(pose_topic_name, 1, &PedTrajPred::posCallback, this); // subscriber to the pose that we want to predict
    pose_with_cov_sub = nh.subscribe(pose_with_cov_topic_name, 1, &PedTrajPred::posCovCallback, this);
    odometry_pose_sub = nh.subscribe(odometry_pose_topic_name, 1, &PedTrajPred::odometryCallback, this);

    ROS_INFO("PedTraPred] -- Listening to %s", pose_topic_name.c_str());

    predict_filter = new PredictFilter();

    predict_filter->setPathId(path_id_param);

    ROS_INFO("[PedTraPred] -- init OK!");

}

PedTrajPred::~PedTrajPred() {
    delete predict_filter;
}

void PedTrajPred::posCallback(const geometry_msgs::PoseStampedConstPtr& pos) {
    // get input position and convert it to the measure vector
    predict_filter->output_frame_id = pos->header.frame_id;
    double time_callback;
    time_callback = pos->header.stamp.toSec();

    if (time_callback - t_prev_measure < predict_filter->getTimeStep()) {
        return;
    }

    predict_filter->refreshPos(pos->pose.position.x, pos->pose.position.y);
    predict_filter->publishPred(pos->pose.position.x, pos->pose.position.y, pos->pose.position.z);
    t_prev_measure = time_callback;
}

void PedTrajPred::odometryCallback(const nav_msgs::OdometryConstPtr& odom) {
    // get input position and convert it to the measure vector
    predict_filter->output_frame_id = odom->header.frame_id;
    double time_callback;
    time_callback = odom->header.stamp.toSec();

    if (time_callback - t_prev_measure < predict_filter->getTimeStep()) {
        return;
    }

    predict_filter->refreshPos(odom->pose.pose.position.x, odom->pose.pose.position.y);
    t_prev_measure =  time_callback;
}

void PedTrajPred::posCovCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& posCov) {
    predict_filter->output_frame_id = posCov->header.frame_id;
    double time_callback;
    time_callback = posCov->header.stamp.toSec();

    if (time_callback - t_prev_measure < predict_filter->getTimeStep()) {
        return;
    }

    predict_filter->refreshPos(posCov->pose.pose.position.x, posCov->pose.pose.position.y);
    t_prev_measure = time_callback;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "predict_step");

    ROS_INFO("[PED TRAJ PRED] - Start node");
    PedTrajPred pos_kal;

    ros::Rate ros_rate(4);
    sleep(2);

    while (pos_kal.predict_filter->filter_initialized == false && ros::ok()) {
        ros::spinOnce();
        ros_rate.sleep();
        ROS_INFO("Waiting for any position message");
    }

    pos_kal.predict_filter->publish_timer.start();
    ros::spin();
}
