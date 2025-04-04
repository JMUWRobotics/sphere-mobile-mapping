/*
 * pcl_kd_tree_test.cpp
 * Testing the pcl tree implementation
 *
 * Author: Carolin Bösch
 */

#include "hough.h"

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
// NEW PLANE
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
// NEW TRANSFORM NORMAL VECTOR TO MAP FRAME
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;
using PointType = pcl::PointXYZ;
using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;


enum PlaneSegm {
    LSF, PCA, RANSAC, RHT
};

class GroundFinder{
private:
    /* Normal vector */
    std::vector<double> n = {0, 0, -1}; // Previous normalized normal vector (n)
    visualization_msgs::Marker n_marker;

    ros::NodeHandle nh;
    ros::Publisher pub_subset;
    ros::Publisher pub_filtered;
    ros::Publisher pub_n;
    ros::Publisher pub_vis_n;
    ros::Subscriber sub;

    // Create a TF2 buffer and listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    tf2_ros::Buffer tf_buffer;

    string type_str;
    PlaneSegm type;

    string filename;
    ofstream csv;

    void scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
        /* Convert PointCloud2 (sensor msg) to PointCloud (pcl) */
        pcl::PointCloud<PointType>::Ptr cur_scan(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cur_scan);

        // Implementation of query point
        float radius_sphere = 0.145;
        PointType query_point = {0.0, 0.0, 0.0};
        query_point.x = n[0] * radius_sphere;
        query_point.y = n[1] * radius_sphere;
        query_point.z = n[2] * radius_sphere;

        /* 1) Filter cloud: Take out reflections from sphere + hands */
        float filter_radius = 0.35; // NOTE: radius sphere incl. noise + hands = 0.35 m (from rviz)
        pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
        auto start_loop = chrono::high_resolution_clock::now();
        // Loop through cloud
        for (int i = 0; i < (*cur_scan).size(); i++){
            PointType p_i = cur_scan->points[i];
            double dist_sqr = pow(p_i.x, 2) + pow(p_i.y, 2) + pow(p_i.z, 2);
            if(dist_sqr <= pow(filter_radius, 2)){
                del_points->indices.push_back(i);
            }
        }
        auto end_loop = chrono::high_resolution_clock::now();
        auto duration_loop = chrono::duration_cast<chrono::microseconds>(end_loop - start_loop).count();
        // ROS_INFO("Looping through cloud takes: %0.6f ms", float(duration_loop)/ 1e3);

        // Delete points from cloud
        pcl::ExtractIndices<PointType> extract;
        auto start_del    = chrono::high_resolution_clock::now();
        extract.setInputCloud(cur_scan);
        extract.setIndices(del_points);
        extract.setNegative(true);
        extract.filter(*cur_scan);
        auto end_del      = chrono::high_resolution_clock::now();
        auto duration_del = chrono::duration_cast<chrono::microseconds>(end_del - start_del).count();
        // ROS_INFO("Delete points from cloud takes: %0.3f ms", float(duration_del) / 1e3);

        auto duration_filter = duration_loop + duration_del;
        ROS_INFO("Filtering takes: %0.6f ms", float(duration_filter)/ 1e3);

        /* Publish filtered cloud */
        sensor_msgs::PointCloud2::Ptr filtered_cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cur_scan, *filtered_cloud_msg);
        filtered_cloud_msg->header.stamp = msg->header.stamp;
        filtered_cloud_msg->header.frame_id = msg->header.frame_id;
        pub_filtered.publish(*filtered_cloud_msg);

        /* 2) Build kd-tree */
        pcl::KdTreeFLANN<PointType> kd_tree;
        auto start_build    = chrono::high_resolution_clock::now();
        kd_tree.setInputCloud(cur_scan);
        auto end_build      = chrono::high_resolution_clock::now();
        auto duration_build = chrono::duration_cast<chrono::microseconds>(end_build - start_build).count();
        ROS_INFO("Building tree takes: %0.6f ms", float(duration_build) / 1e3);

        /* 3) Search kNN */
        int k = 1000; //NOTE: 2500 bit much even for ransac!
        if(type == RANSAC) k = 10000;
        // Return values
        vector<int> search_result;
        vector<float> search_distances;
        // kNN search
        auto start_search    = chrono::high_resolution_clock::now();
        kd_tree.nearestKSearch(query_point, k, search_result, search_distances);
        auto end_search      = chrono::high_resolution_clock::now();
        auto duration_search = chrono::duration_cast<chrono::microseconds>(end_search - start_search).count();
        ROS_INFO("Search kNN takes: %0.6f ms", float(duration_search) / 1e3);

        // Write to csv file
        // csv << duration_build << "," << duration_search << "," << duration_total << "\n";

        /* Create Point Cloud from search result (sub_cloud) and publish it */
        pcl::ExtractIndices<PointType> extract2;
        pcl::PointIndices::Ptr sub_points(new pcl::PointIndices());
        sub_points->indices = search_result;
        extract2.setInputCloud(cur_scan);
        extract2.setIndices(sub_points);
        extract2.filter(*cur_scan);

        sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cur_scan, *sub_cloud_msg);
        sub_cloud_msg->header.stamp = msg->header.stamp;
        sub_cloud_msg->header.frame_id = msg->header.frame_id;
        pub_subset.publish(*sub_cloud_msg);


        /* --------------------------------- PLANE DETECTION ---------------------------------*/

        int64_t duration_plane = 0.0;
        switch(type){
            case LSF:{
                float curv;
                Eigen::Vector4f params;
                auto start_plane = chrono::high_resolution_clock::now();
                pcl::computePointNormal(*cur_scan, params, curv); // NOTE: eigenvector = already normalized :)
                // Set new normal vector
                n[0] = params[0];
                n[1] = params[1];
                n[2] = params[2];
                auto end_plane = chrono::high_resolution_clock::now();
                duration_plane = chrono::duration_cast<chrono::microseconds>(end_plane - start_plane).count();
                break;
            }
            case PCA:{
                pcl::PCA<PointType> pca;
                auto start_plane = chrono::high_resolution_clock::now();
                pca.setInputCloud(cur_scan);
                Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
                // Set new normal vector
                n[0] = eigen_vecs.coeff(0, 2);
                n[1] = eigen_vecs.coeff(1, 2);
                n[2] = eigen_vecs.coeff(2, 2);
                auto end_plane = chrono::high_resolution_clock::now();
                duration_plane = chrono::duration_cast<chrono::microseconds>(end_plane - start_plane).count();
                break;
            }
            case RANSAC:{
                pcl::PointCloud<PointType>::Ptr final(new pcl::PointCloud<PointType>);
                vector<int> inliers;
                // Created RandomSampleConsensus object and compute the appropriated model
                pcl::SampleConsensusModelPlane<PointType>::Ptr model(new pcl::SampleConsensusModelPlane<PointType>(cur_scan));
                pcl::RandomSampleConsensus<PointType> ransac(model);
                auto start_plane = chrono::high_resolution_clock::now();
                ransac.setDistanceThreshold(.01);
                ransac.computeModel();
                ransac.getInliers(inliers);
                // Copies all inliers of the model computed to another PointCloud
                pcl::copyPointCloud(*cur_scan, inliers, *final);

                // Normal vector calculation LSF // TODO oder PCA?? mit pca -> dann no copyPointcloud aber mit indices (inliers)!
                float curv;
                Eigen::Vector4f params;
                pcl::computePointNormal(*final, params, curv); // NOTE: eigenvector = already normalized :)
                // Set new normal vector
                n[0] = params[0];
                n[1] = params[1];
                n[2] = params[2];

                auto end_plane = chrono::high_resolution_clock::now();
                duration_plane = chrono::duration_cast<chrono::microseconds>(end_plane - start_plane).count();
                break;
            }
            case RHT:{
                // Accumulator //TODO define params! - now from paper (dorit) and config file
                int rhoNum = 100, phiNum = 45, thetaNum = 90, rhoMax = 600, accumulatorMax = 1000;
                Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
                // Hough object
                double minDist = 0.25; // min distance between points (50m = to big for our case; 0.25m)
                double maxDist = 10.0; // max distance between points (500m = to big for our case; 10.0m)
                Hough hough(cur_scan, &acc, minDist, maxDist);

                auto start_plane = chrono::high_resolution_clock::now();
                hough.RHT(n); // NOTE: eigenvector = already normalized :)
                auto end_plane = chrono::high_resolution_clock::now();
                duration_plane = chrono::duration_cast<chrono::microseconds>(end_plane - start_plane).count();
                break;
            }
        }
        ROS_INFO("Plane Segmentation takes: %0.6f ms", float(duration_plane) / 1e3);
        ROS_INFO("Normal Vector: nx = %.3f, ny: %.3f, nz: %.3f", n[0], n[1], n[2]);

        /* Total time for kd_tree build + filter + kNN ranged search + plane segmentation */
        auto duration_total = duration_filter + duration_build + duration_search + duration_plane;
        ROS_WARN("Total time: %0.6f ms\n", float(duration_total) / 1e3);

        /* Convert normal vector to map2 frame */
        // Listen to tf tree for transformation
        geometry_msgs::TransformStamped t;
        try {
            t = tf_buffer.lookupTransform("map2", "pandar_frame", ros::Time(0));
        } catch(tf2::TransformException& ex){
            ROS_ERROR("Failed to listen to tf tree!\n\n");
            return;
        }
        // Create Vector n in pandar frame
        // TODO: change std::vector to Vector Stamped also for math helpers? here: saves conversion
        geometry_msgs::Vector3Stamped n_pandar;
        n_pandar.header.frame_id = msg->header.frame_id;
        n_pandar.vector.x = n[0];
        n_pandar.vector.y = n[1];
        n_pandar.vector.z = n[2];
        // Create Vector n in map2 frame (to be published)
        geometry_msgs::Vector3Stamped n_msg;
        n_msg.header.frame_id = msg->header.frame_id;
        n_msg.header.stamp = msg->header.stamp;
        tf2::doTransform(n_pandar, n_msg, t);

        /* Make sure it always points into ground */
        // NOTE: Assumption = slope of ground plane < 90°
        std::vector<double> down = {0.0, 0.0, -1.0};
        std::vector<double> n_map2 = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
        // The dot product = positive if the angle between both vectors is smaller than 90 degrees, and negative otherwise.
        if(dot_product(down, n_map2) < 0){
            n[0] = -n[0];
            n[1] = -n[1];
            n[2] = -n[2];
            n_msg.vector.x = -n_msg.vector.x;
            n_msg.vector.y = -n_msg.vector.y;
            n_msg.vector.z = -n_msg.vector.z;
        }

        /* Publish normal vector */
        pub_n.publish(n_msg);

        /* Update normal vector n_marker */
        n_marker.header.frame_id = msg->header.frame_id;
        n_marker.header.stamp = msg->header.stamp;
        geometry_msgs::Point start, end;
        start.x = query_point.x;
        start.y = query_point.y;
        start.z = query_point.z;
        end.x = (start.x + n[0])/3.0;
        end.y = (start.y + n[1])/3.0;
        end.z = (start.z + n[2])/3.0;
        n_marker.points = {start, end};
        // Publish n_marker
        pub_vis_n.publish(n_marker);
    }

public:
    GroundFinder(){
        /* Read out parameters */
        // ROS_INFO("Checking for param: %s", nh.hasParam("/pcl_kd_test_node/filter") ? "true" : "false");
        if(!nh.getParam("/plane_test_node/type", type_str)){
            ROS_ERROR("Parameter failure... Ending Node!");
            ros::shutdown();
        } else {
            ROS_INFO("Using plane segmentation alg.: %s", type_str.c_str());
            if(!strcmp(type_str.c_str(), "lsf")){
                type = LSF;
            } else if(!strcmp(type_str.c_str(), "pca")){
                type = PCA;
            } else if(!strcmp(type_str.c_str(), "ran")){
                type = RANSAC;
            } else if(!strcmp(type_str.c_str(), "rht")){
                type = RHT;
            } else {
                ROS_ERROR("Invalid Parameter type! Valid options are: 'lsf', 'pca', 'ran' or 'rht'. Ending Node!");
                ros::shutdown();
            }
        }

        // if(!nh.getParam("/plane_test_node/file", filename)){
        //     ROS_ERROR("Parameter failure... Ending Node!");
        //     ros::shutdown();
        // } else {
        //     if(kd_filtering){
        //         ROS_INFO("Writing to file: data/pcl/kd-filter%s.csv\n", filename.c_str());
        //     } else {
        //         ROS_INFO("Writing to file: data/pcl/geo-filter%s.csv\n", filename.c_str());
        //     }
        // }

        /* Initialize Topics */
        pub_subset   = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/sub_cloud", 1);
        pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/filtered_cloud", 1);
        pub_n   = nh.advertise<geometry_msgs::Vector3Stamped>("/ground_finder/normal_vector", 1);
        pub_vis_n  = nh.advertise<visualization_msgs::Marker>("/ground_finder/normal_marker", 1);
        // tf_buffer = std::make_unique<tf2_ros::Buffer>();
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
        ROS_WARN("Waiting for TF Listener!");
        bool tf_listener_fail = true;
        while(tf_listener_fail){
            try {
                tf_listener_fail = false;
                geometry_msgs::TransformStamped t = tf_buffer.lookupTransform("map2", "pandar_frame", ros::Time(0));
            } catch(tf2::TransformException& ex){
                tf_listener_fail = true;
                ros::Duration(0.5).sleep();
            }
        }
        ROS_INFO("Received Transformation! Starting subscriber now!\n");

        /* Init n_marker with default values */
        n_marker.type = visualization_msgs::Marker::ARROW;
        n_marker.action = visualization_msgs::Marker::ADD;
        n_marker.pose.orientation.x = 0.0;
        n_marker.pose.orientation.y = 0.0;
        n_marker.pose.orientation.z = 0.0;
        n_marker.pose.orientation.w = 1.0;
        n_marker.scale.x = 0.02;
        n_marker.scale.y = 0.06;
        n_marker.scale.z = 0.06;
        n_marker.color.a = 1.0;
        n_marker.color.r = 1.0;
        n_marker.color.g = 0.0;
        n_marker.color.b = 0.0;

        /* Subscribe to Hesai laser scanner */
        sub = nh.subscribe("/hesai/pandar", 1, &GroundFinder::scan_callback, this);

        // /* Open file stream */
        // string path;
        // if(kd_filtering){
        //     path = "/home/caro/catkin_ws/src/ground_finder/data/pcl/kd-filter" + filename + ".csv";
        // } else {
        //     path = "/home/caro/catkin_ws/src/ground_finder/data/pcl/geo-filter" + filename + ".csv";
        // }
        // csv.open(path);
        // // Write header
        // csv << "FilterTimeTotal[ns],BuildDelTree[ns],SearchDel[ns],Delete[ns],BuildTree[ns],SearchKNN[ns],Total[ns]\n";
    }
};

int main(int argc, char** argv){
    /* Init ROS */
    ros::init(argc, argv, "ground_finder");
    printf("\n---------------------------------------\n\tSTARTING PLANE TEST!\n---------------------------------------\n");

    /* Create object of class GroundFinder */
    GroundFinder ground_finder;

    /* Loop */
    ros::Rate freq(50);
    ros::spin();
    freq.sleep();

    return 0;
}
