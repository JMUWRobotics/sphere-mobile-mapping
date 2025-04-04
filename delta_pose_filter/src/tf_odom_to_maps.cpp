#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <state_estimator_msgs/Estimator.h>

#include <queue>
#include <mutex>

// Rosparam parameters
const char* topic_default = "/posePub_merged";
const char* global_frame_default = "map";
const char* center_frame_default = "odom";
std::string global_frame, center_frame;

void sendPoseStampedAsInverseTransform(const geometry_msgs::PoseStamped &m, std::string frame, std::string child_frame)
{
    // Static Transform Map -> IMU
    static tf::TransformBroadcaster br;
	tf::Transform transform; 

    transform.setOrigin(
        tf::Vector3(
            m.pose.position.x,
            m.pose.position.y,
            m.pose.position.z
        ) 
    );
    tf::Quaternion quat;
    tf::quaternionMsgToTF(m.pose.orientation, quat);
    transform.setRotation(quat);        
    br.sendTransform(tf::StampedTransform(transform.inverse(), m.header.stamp, frame, child_frame));
}

void poseMsgCallback(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    sendPoseStampedAsInverseTransform(m->pose, center_frame, global_frame);
}

void flawedMsgCallback(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    sendPoseStampedAsInverseTransform(m->pose, "odom", "map1");
}

void lkfMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &m)
{
    sendPoseStampedAsInverseTransform(*m, "odom", "map2");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_maps_publisher");
    ros::NodeHandle nh;

    int spinrate = 1000; // Hz
    // Topic params 
    std::string topic_listen;
    nh.param<std::string>("topic_listen", topic_listen, std::string(topic_default)); 
    nh.param<std::string>("global_frame", global_frame, std::string(global_frame_default));
    nh.param<std::string>("center_frame", center_frame, std::string(center_frame_default));
    // Publishers and subscribers
    ros::Subscriber imu_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_listen, 1000, poseMsgCallback);
    ros::Subscriber flawed_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_listen + "_raw", 1000, flawedMsgCallback);
    ros::Subscriber lkf_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/lkf/pose", 1000, lkfMsgCallback);
    // Main processing loop, wait for callbacks to happen
    ros::Rate rate(spinrate);
    while(ros::ok()) {
        ros::spinOnce();
	    rate.sleep();
    }

    return 0;
}


