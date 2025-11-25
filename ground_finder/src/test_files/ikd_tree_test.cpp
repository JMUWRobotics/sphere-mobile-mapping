/*
 * ikd_tree_test.cpp
 * Testing the ikd tree implementation from hku mars
 *
 * Author: Carolin Bösch
 */

#include <cmath>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include "ikd_tree/ikd_Tree.h"

using namespace std;
using PointType = pcl::PointXYZ;
using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;

/* Notes:
 * - copy point cloud after filtering for plane segmentation: pcl::copyPointCloud (*cloud, inliers, *final);
 */

class GroundFinder{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_subset;
    ros::Publisher pub_filtered;
    ros::Subscriber sub;

    bool kd_filtering;
    string filename;
    ofstream csv;

    void scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
        /* Convert PointCloud2 (sensor msg) to PointCloud (pcl) */
        // pcl::PCLPointCloud2 tmp_pc;
        // pcl_conversions::toPCL(*msg, tmp_pc);
        pcl::PointCloud<PointType>::Ptr cur_scan(new pcl::PointCloud<PointType>);
        // pcl::fromPCLPointCloud2(tmp_pc,*cur_scan);
        pcl::fromROSMsg(*msg, *cur_scan);

        /* Initialize kd-tree */
        KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.5, 0.6, 0.2));  // NOTE: delete param up to 0.5 nothing, balance param to 0.9 = better, boxlength to 0.5 = nothing
        KD_TREE<PointType>      &kd_tree        = *kdtree_ptr;

        /* 1) Filter cloud: Take out reflections from sphere + hands incl. kd-tree management */
        // Helper variables
        int64_t duration_filter;
        float radius_sphere = 0.35; // NOTE: radius sphere = 0.2 m + hands = 0.35 m (from rviz)
        pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
        pcl::ExtractIndices<PointType> extract;
        pcl::PointIndices::Ptr del_points(new pcl::PointIndices());

        /* 1A) Filtering using kd-tree and radius search + Incremental update (delete points) */
        if(kd_filtering){
            // Build balanced kd-tree from point cloud
            auto start_build = chrono::high_resolution_clock::now();
            kd_tree.Build((*cur_scan).points);
            auto end_build      = chrono::high_resolution_clock::now();
            auto duration_build = chrono::duration_cast<chrono::microseconds>(end_build - start_build).count();
            ROS_INFO("Building tree takes: %0.3f ms", float(duration_build) / 1e3);

            // Radius search
            PointType origin;
            origin.x = 0.0;
            origin.y = 0.0;
            origin.z = 0.0;
            PointVector search_radius_result;
            auto start_search    = chrono::high_resolution_clock::now();
            kd_tree.Radius_Search(origin, radius_sphere, search_radius_result);
            auto end_search      = chrono::high_resolution_clock::now();
            auto duration_search = chrono::duration_cast<chrono::microseconds>(end_search - start_search).count();
            ROS_INFO("Radius search takes: %0.3f ms", float(duration_search) / 1e3);

            // Delete points from kd-tree
            auto start_del    = chrono::high_resolution_clock::now();
            kd_tree.Delete_Points(search_radius_result);
            auto end_del      = chrono::high_resolution_clock::now();
            auto duration_del = chrono::duration_cast<chrono::microseconds>(end_del - start_del).count();
            ROS_INFO("Delete points from ikd-tree takes: %0.3f ms", float(duration_del) / 1e3);

            // "Create" filtered cloud
            filtered_cloud->points = kd_tree.PCL_Storage;

            duration_filter = duration_build + duration_search + duration_del;
            ROS_INFO("-- Total filtering takes: %0.6f ms", float(duration_filter)/ 1e3);

            // Write to csv file
            csv << duration_filter << "," << duration_build << "," << duration_search << "," << duration_del << ",";

        /* 1B) Filtering by loping through entire point cloud then building kd-tree from filtered cloud */
        } else {
            auto start_loop = chrono::high_resolution_clock::now();
            // Loop through cloud
            for (int i = 0; i < (*cur_scan).size(); i++){
                PointType p_i = cur_scan->points[i];
                double dist_sqr = pow(p_i.x, 2) + pow(p_i.y, 2) + pow(p_i.z, 2);
                if(dist_sqr <= pow(radius_sphere, 2)){
                    del_points->indices.push_back(i);
                }
            }
            auto end_loop = chrono::high_resolution_clock::now();
            auto duration_loop = chrono::duration_cast<chrono::microseconds>(end_loop - start_loop).count();
            ROS_INFO("Looping through cloud takes: %0.6f ms", float(duration_loop)/ 1e3);

            // Delete points from cloud
            auto start_del    = chrono::high_resolution_clock::now();
            extract.setInputCloud(cur_scan);
            extract.setIndices(del_points);
            extract.setNegative(true);
            extract.filter(*filtered_cloud);
            auto end_del      = chrono::high_resolution_clock::now();
            auto duration_del = chrono::duration_cast<chrono::microseconds>(end_del - start_del).count();
            ROS_INFO("Delete points from cloud takes: %0.3f ms", float(duration_del) / 1e3);

            // Build balanced kd-tree from filtered point cloud
            auto start_build = chrono::high_resolution_clock::now();
            kd_tree.Build((*filtered_cloud).points);
            auto end_build      = chrono::high_resolution_clock::now();
            auto duration_build = chrono::duration_cast<chrono::microseconds>(end_build - start_build).count();
            ROS_INFO("Building tree takes: %0.3f ms", float(duration_build) / 1e3);

            duration_filter = duration_loop + duration_del + duration_build;
            ROS_INFO("-- Total filtering takes: %0.6f ms", float(duration_filter)/ 1e3);

            // Write to csv file
            csv << duration_filter << "," << duration_build << "," << duration_loop << "," << duration_del << ",";
        }

        /* Publish filtered cloud */
        sensor_msgs::PointCloud2::Ptr filtered_cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*filtered_cloud, *filtered_cloud_msg);
        filtered_cloud_msg->header.stamp = msg->header.stamp;
        filtered_cloud_msg->header.frame_id = msg->header.frame_id;
        pub_filtered.publish(*filtered_cloud_msg);

        /* 2) Search kNN */
        // Search param: k
        int k = 1000;
        // Search param: query point
        PointType query_point;
        query_point.x = 0.0;
        query_point.y = 0.0;
        query_point.z = 0.0;
        // Search param: max distance
        double max_dist = 2.0;
        // Return values
        PointVector search_result;
        vector<float> PointDist;
        // Ranged search
        auto start_search    = chrono::high_resolution_clock::now();
        kd_tree.Nearest_Search(query_point, k, search_result, PointDist, max_dist);
        auto end_search      = chrono::high_resolution_clock::now();
        auto duration_search = chrono::duration_cast<chrono::microseconds>(end_search - start_search).count();
        ROS_INFO("Search kNN takes: %0.6f ms", float(duration_search) / 1e3);

        /* Total time for kd_tree build + filter + kNN ranged search */
        auto duration_total = duration_filter + duration_search;
        ROS_INFO("Total time: %0.6f ms \n", float(duration_total) / 1e3);

        // Write to csv file
        csv << duration_search << "," << duration_total << "\n";

        /* Create Point Cloud from search result (sub_cloud) and publish it */
        pcl::PointCloud<PointType>::Ptr sub_cloud(new pcl::PointCloud<PointType>);
        sub_cloud->points = search_result;
        sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*sub_cloud, *sub_cloud_msg);
        sub_cloud_msg->header.stamp = msg->header.stamp;
        sub_cloud_msg->header.frame_id = msg->header.frame_id;
        pub_subset.publish(*sub_cloud_msg);
    }

public:
    GroundFinder(){
        /* Read out parameters */
        // ROS_INFO("Checking for param: %s", nh.hasParam("/ikd_test_node/filter") ? "true" : "false");
        if(!nh.getParam("/ikd_test_node/filter", kd_filtering)){
            ROS_ERROR("Parameter failure... Ending Node!");
            ros::shutdown();
        } else {
            ROS_INFO("Using kd-tree for filtering: %s", kd_filtering ? "true" : "false");
        }

        if(!nh.getParam("/ikd_test_node/file", filename)){
            ROS_ERROR("Parameter failure... Ending Node!");
            ros::shutdown();
        } else {
            if(kd_filtering){
                ROS_INFO("Writing to file: data/ikd/kd-filter%s.csv\n", filename.c_str());
            } else {
                ROS_INFO("Writing to file: data/ikd/geo-filter%s.csv\n", filename.c_str());
            }
        }

        /* Initialize Topics */
        pub_subset   = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/sub_cloud", 1);
        pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/filtered_cloud", 1);

        /* Subscribe to Hesai laser scanner */
        sub = nh.subscribe("/hesai/pandar", 1, &GroundFinder::scan_callback, this);

        /* Open file stream */
        string path;
        if(kd_filtering){
            path = "/home/caro/catkin_ws/src/ground_finder/data/ikd/kd-filter" + filename + ".csv";
        } else {
            path = "/home/caro/catkin_ws/src/ground_finder/data/ikd/geo-filter" + filename + ".csv";
        }
        csv.open(path);
        // Write header
        csv << "FilterTimeTotal[ns],BuildTree[ns],SearchDel[ns],Delete[ns],SearchKNN[ns],Total[ns]\n";

    }
};

int main(int argc, char** argv){
    /* Init ROS */
    ros::init(argc, argv, "ground_finder");
    printf("\n---------------------------------------\n\tSTARTING IKD_TREE TEST!\n---------------------------------------\n");

    /* Create object of class GroundFinder */
    GroundFinder ground_finder;

    /* Loop */
    ros::Rate freq(50);
    ros::spin();
    freq.sleep();

    return 0;
}
