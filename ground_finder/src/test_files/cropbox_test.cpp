/*
 * cropbox_test.cpp
 * Testing the cropbox filter for subcloud creation
 *
 * Author: Carolin Bösch
 */

#include "math_helpers.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// TF2 libs
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using PointType = pcl::PointXYZ;

class GroundFinder{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_subset;
    ros::Subscriber sub;

    /* TF2 variables */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr}; // Transform listener
    tf2_ros::Buffer tf_buffer;    // Transform buffer

    void downsample(pcl::PointCloud<PointType>::Ptr &cur_scan){
        pcl::VoxelGrid<PointType> ds;
        ds.setInputCloud(cur_scan);
        ds.setLeafSize(0.1f, 0.1f, 0.1f);
        ds.filter(*cur_scan);
    }

    void scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
        /* Convert PointCloud2 (sensor msg) to PointCloud (pcl) */
        pcl::PointCloud<PointType>::Ptr cur_scan(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cur_scan);

        auto start = std::chrono::high_resolution_clock::now();

        /* Downsample cloud */
        downsample(cur_scan);

        // Helper variables
        pcl::ExtractIndices<PointType> extract;
        pcl::PointIndices::Ptr del_points(new pcl::PointIndices());

        /* 1) Filter geo cloud */
        float radius_sphere = 0.35; // NOTE: radius sphere = 0.2 m + hands = 0.35 m (from rviz)
        // Loop through cloud
        for (int i = 0; i < (*cur_scan).size(); i++){
            PointType p_i = cur_scan->points[i];
            double dist_sqr = pow(p_i.x, 2) + pow(p_i.y, 2) + pow(p_i.z, 2);
            if(dist_sqr <= pow(radius_sphere, 2)){
                del_points->indices.push_back(i);
            }
        }
        // Delete points from cloud
        extract.setInputCloud(cur_scan);
        extract.setIndices(del_points);
        extract.setNegative(true);
        extract.filter(*cur_scan);

        /* OLD: GEO approach for subcloud */
        // pcl::PointIndices::Ptr points(new pcl::PointIndices());
        // float radius_subcloud = 2.0;
        // // Loop through cloud
        // auto start_geo = std::chrono::high_resolution_clock::now();
        // for (int i = 0; i < (*cur_scan).size(); i++){
        //     PointType p_i = cur_scan->points[i];
        //     double d_sqr = pow(p_i.x, 2) + pow(p_i.y, 2) + pow(p_i.z, 2);
        //     if(d_sqr <= pow(radius_subcloud, 2)){
        //         points->indices.push_back(i);
        //     }
        // }

        // // Delete points from cloud
        // extract.setInputCloud(cur_scan);
        // extract.setIndices(points);
        // extract.filter(*cur_scan);

        // auto end_geo = std::chrono::high_resolution_clock::now();
        // auto duration_geo = std::chrono::duration_cast<std::chrono::microseconds>(end_geo - start_geo).count();
        // ROS_INFO("Creating subcloud takes: %0.3f ms\n", float(duration_geo) / 1e3);

        /* NEW: Cropbox filter for subcloud */
        pcl::CropBox<PointType> crop;

        auto start_sub    = chrono::high_resolution_clock::now();
        geometry_msgs::TransformStamped t;
        try {
            // Listen to tf tree for transformation
            t = tf_buffer.lookupTransform("pandar_frame", "map2", ros::Time(0));
        } catch(tf2::TransformException& ex){
            ROS_ERROR("Failed to listen to tf tree!\n\n");
            return;
        }
        // Create "down" Vector n in map2 frame
        geometry_msgs::Vector3Stamped n_map;
        n_map.vector.x = 1.0;
        n_map.vector.y = 1.0;
        n_map.vector.z = 0.05;
        geometry_msgs::Vector3Stamped n_pandar;
        tf2::doTransform(n_map, n_pandar, t);
        // ROS_WARN("x = %.3f, y = %.3f and z = %.3f", n_pandar.vector.x, n_pandar.vector.y, n_pandar.vector.z);

        Eigen::Vector4f min(-fabs(n_pandar.vector.x), -fabs(n_pandar.vector.y), -fabs(n_pandar.vector.z), 0);
        Eigen::Vector4f max( fabs(n_pandar.vector.x),  fabs(n_pandar.vector.y),  fabs(n_pandar.vector.z), 0);

        crop.setInputCloud(cur_scan);
        crop.setMin(min);
        crop.setMax(max);
        crop.filter(*cur_scan);
        auto end_sub      = chrono::high_resolution_clock::now();
        auto duration_sub = chrono::duration_cast<chrono::microseconds>(end_sub - start_sub).count();
        ROS_INFO("Creating subcloud takes: %0.3f ms\n", float(duration_sub) / 1e3);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
        ROS_INFO("All: %0.3f ms\n", float(duration) / 1e3);

        // Publish filtered subcloud
        sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cur_scan, *sub_cloud_msg);
        sub_cloud_msg->header.stamp = msg->header.stamp;
        sub_cloud_msg->header.frame_id = msg->header.frame_id;
        pub_subset.publish(*sub_cloud_msg);
    }

public:
    GroundFinder(){
        /* Initialize Topics */
        pub_subset   = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/sub_cloud", 1);

        // Wait for first transform (pandar_frame to map2)
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
        ROS_WARN("Waiting for TF Listener!");
        bool tf_listener_fail = true;
        while(tf_listener_fail && ros::ok()){
            try {
                tf_listener_fail = false;
                geometry_msgs::TransformStamped t = tf_buffer.lookupTransform("map2", "pandar_frame", ros::Time(0));
            } catch(tf2::TransformException& ex){
                tf_listener_fail = true;
                ros::Duration(0.1).sleep();
            }
        }
        ROS_INFO("Received Transformation! Starting subscriber now!\n");

        /* Subscribe to Hesai laser scanner */
        sub = nh.subscribe("/lidar/points_undistorted", 1, &GroundFinder::scan_callback, this);
    }
};

int main(int argc, char** argv){
    /* Init ROS */
    ros::init(argc, argv, "ground_finder");
    printf("\n---------------------------------------\n\tSTARTING CROP_BOX TEST!\n---------------------------------------\n");

    /* Create object of class GroundFinder */
    GroundFinder ground_finder;

    /* Loop */
    ros::Rate freq(50);
    ros::spin();
    freq.sleep();

    return 0;
}
