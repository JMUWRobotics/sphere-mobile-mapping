// Smart pointers
#include <memory>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

// Custom msg
#include <state_estimator_msgs/Estimator.h>
typedef state_estimator_msgs::Estimator CamPose;
typedef state_estimator_msgs::EstimatorConstPtr CamPosePtr;

// ROS topic publisher
ros::Publisher center_pose_pub;
CamPose center_pose_msg;

// Listen to transform
tf::StampedTransform tf_axes_imu2cam;

// Extrinsic parameters FROM sphere center TO cam 
tf::Vector3 calib;

inline bool isNormalized(const geometry_msgs::Quaternion &q)
{
    return fabs(sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) - 1.0) < 0.0001;
}

void cameraPoseCallback(const CamPosePtr &m)
{
    // Check if orientation quaternion normalized
    if (!isNormalized(m->pose.pose.orientation)) return;
    
    // Convert orientation MSG to Rotation Matrix
    tf::Quaternion rotation;
    tf::quaternionMsgToTF(m->pose.pose.orientation, rotation);
    tf::Matrix3x3 rotm(rotation);

    // Calculate center position of sphere
    tf::Vector3 offset = rotm * calib;
    double center_pos_x = m->pose.pose.position.x - offset.x();
    double center_pos_y = m->pose.pose.position.y - offset.y();
    double center_pos_z = m->pose.pose.position.z - offset.z();

    // Construct published topic msg (where the sphere center point should be according to cam)
    center_pose_msg.header = m->header;
    center_pose_msg.imu = m->imu;
    center_pose_msg.pose = m->pose;
    center_pose_msg.pose.pose.position.x = center_pos_x;
    center_pose_msg.pose.pose.position.y = center_pos_y;
    center_pose_msg.pose.pose.position.z = center_pos_z;
    center_pose_msg.tracker_confidence = m->tracker_confidence;
    center_pose_pub.publish(center_pose_msg);
}

int main(int argc, char **argv)
{
    // Init nodehandle
    ros::init(argc, argv, "t265_extrinsics_node");
    ros::NodeHandle nh;

    // Get the extrinsic calibration
    double x, y, z;
    nh.param<double>("offset_x", x, 0);
    nh.param<double>("offset_y", y, 0);
    nh.param<double>("offset_z", z, 0);
    
    // Get static tf between global and camera frame
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), ros::Duration(5.0));             
    tf_listener.lookupTransform(std::string("axes_cam"), std::string("axes_imu"), ros::Time(0), tf_axes_imu2cam);
    
    // We give the extr calib in GLOBAL frame, need to transfer to camera_frame first 
    tf::Matrix3x3 R(tf_axes_imu2cam.getRotation());
    calib.setX(x);
    calib.setY(y);
    calib.setZ(z);
    calib = calib * R;

    // Setup subscriber loop
    center_pose_pub = nh.advertise<CamPose>("/camera/sphereCenterPose", 1000);
    ros::Subscriber cam_pose_sub = nh.subscribe<CamPose>("/camera/poseAndImu", 1000, cameraPoseCallback);
    ros::spin();
    return 0;
}