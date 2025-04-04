/*
 * ground_finder.h
 * Real-time calculation of the ground plane's normal vector of the hesai pandar scanner
 *
 * Author: Carolin Bösch
 */

#ifndef GROUND_FINDER_H
#define GROUND_FINDER_H

// Own header files
#include "math_helpers.h"
#include "hough.h"
// System
#include <chrono>
// ROS
#include <ros/ros.h>
// Message types used
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
// PCL libs
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
// TF2 libs
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Preprocessing types
enum Preprocessing {
    NONE, GEOMETRICAL, KD_TREE
};

// Plane segmentation types
enum PlaneSegm {
    LSF, PCA, RANSAC, RHT, RHT2
};

// ---------------------- GroundFinder class ----------------------
class GroundFinder {
private:

    int count_fail;
    int plane_counter;

    /* Normal calculation */
    std::vector<double> n = {0.0, 0.0, -1.0}; // Normalized normal vector (n) in pandar_frame
    visualization_msgs::Marker n_marker;

    /* ROS variables for node */
    ros::NodeHandle nh;           // Node handle
    ros::Subscriber sub_h;        // Subscriber for hesai topic
    ros::Subscriber sub_p;        // Subscriber for plane topic
    ros::Publisher  pub_subcloud; // Publisher of subcloud
    ros::Publisher  pub_test;     // TODO take out!
    ros::Publisher  pub_test2;    // TODO take out!
    ros::Publisher  pub_n;        // Publisher of normal vector in map2 frame
    ros::Publisher  pub_vis_n;    // Publisher of normal vector marker for rviz

    /* TF2 variables */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr}; // Transform listener
    tf2_ros::Buffer tf_buffer;    // Transform buffer

    /* Preprocessing steps */
    Preprocessing filtering; // Filter type
    Preprocessing subcloud;  // Subcloud type

    /* Plane segmentation */
    PlaneSegm plane_alg;    // Algorithm used for plane segmentation

    /* Writing to file varibales */
    std::string filename; // Filename of csv
    std::ofstream csv;    // File stream
    bool write2file;

    /* Output regulation */
    bool quiet;

    /* Constants */
    const float radius_sphere = 0.145; // radius sphere used for query point calculation
    const float radius_filter = 0.35;  // radius sphere + noise + hands = 0.35 m (from rviz) used for filtering
    const float radius_subcloud = 2.0; // radius used for geometrical subcloud
    const float height_subcloud = 0.2; // height used for geometrical subcloud
    const float wall_thresh = 0.707;   // Threshold used to determine when we recognized a wall; cos(max_angle 70) = wall_threshold
    const float ds_size = 0.10;        // leaf size used for downsampling after filtering
    const int k = 150;                 // k used for kNN for kd tree subcloud //TODO: 150 = for ransac; 50 = better for rht
    const int max_iterations_plane_detection = 3;

    // ---------------------- Init functions  ----------------------
    /** \brief Initalizes values of n_marker for visualization of normal vector in rviz
     */
    void initMarker();

    // ---------------------- Helper functions  ----------------------
    /** \brief Delete points from point cloud \a cur_scan based on given indices
     * \param[out] cur_scan Pointer to the point cloud
     * \param[in] indices   List of points that are part of \a cur_scan
     * \param[in] negativ   False (Default): removes the points from cloud, which are not part of \a indices; True: removes the points of \a indices from cloud
     * \return The time [ns] needed for the delete operation.
     */
    int64_t delete_points(pcl::PointCloud<PointType>::Ptr &cur_scan, pcl::PointIndices::Ptr &indices, const bool negativ = false);


    // ---------------------- Filter cloud ----------------------
    /** \brief Downsamples the point cloud using a VoxelGrid filter
     * \param[in] cur_scan Pointer to the point cloud to be downsampled
     * \return The time [ns] needed for the downsample operation.
     */
    int64_t downsample(pcl::PointCloud<PointType>::Ptr &cur_scan);

    /** \brief Filtering the point cloud (removal of reflections and hands) by looping through entire \a cur_scan and using geometry
     * \param[out] cur_scan Pointer to the point cloud
     * \return The time [ns] needed for the filter operation.
     */
    int64_t filter_cloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan);

    /** \brief Filtering the point cloud (removal of reflections and hands) by using kd-tree and radius search
     * \param[out] cur_scan Pointer to the point cloud
     * \return The time [ns] needed for the filter operation.
     */
    int64_t filter_cloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan);


    // ---------------------- Create subcloud ----------------------
    /** \brief Create subcloud (reduction to points that possibly represent ground plane) by looping through entire \a cur_scan and using
     * geometry
     * \param[out] cur_scan Pointer to the point cloud
     * \param[in] query_point Defines center point of the box
     * \return The time [ns] needed for creating the subcloud.
     */
    int64_t create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point);

    /** \brief Create subcloud (reduction to points that possibly represent ground plane) by using kd-tree and knn search
     * \param[out] cur_scan    Pointer to the point cloud
     * \param[in] k            Number of points from subcloud (find k-Nearest Neighbors)
     * \param[in] query_point  Defines the query point used for knn search
     * \return The time [ns] needed for creating the subcloud.
     */
    int64_t create_subcloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan, int k, PointType query_point);


    // ---------------------- Filter + create subcloud in one (geo) ----------------------
    /** \brief Filter + create subcloud (reduction to points that possibly represent ground plane withoud reflections and hands) by looping once
     * through entire \a cur_scan and using geometry
     * \param[out] cur_scan   Pointer to the point cloud
     * \param[in] query_point Defines center point of the box
     * \return The time [ns] needed for filtering and creating the subcloud.
     */
    int64_t filter_create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point);


    // ---------------------- Normal Vector calculation  ----------------------
    /** \brief Segments the ground plane from the subcloud \a cur_scan and calculates the normal vector of it (pandar_frame)
     * \param[out] cur_scan Pointer to the point cloud
     * \param[out] n        Normal vector of ground plane
     * \param[in]  type     Defines the plane algorithm used for ground plane segmentation
     * \return The time [ns] needed for plane segmentation and normal vector calculation.
     */
    int64_t determine_n_ground_plane(pcl::PointCloud<PointType>::Ptr &cur_scan, PlaneSegm type, geometry_msgs::Vector3Stamped &n_msg);

    /** \brief Converts the normal vector n from pandar_frame to the map2 frame, while ensuring that n always points into ground.
     * \param[out] n_msg n_msg The normal vector message transformed into the map2 frame
     * \param[in]  last_iteration True (Default): Last try for plane segmentation and overwrites n to [0,0,-1] in map2 frame if wall found;
     * False: Does not overwrite n vector;
     * \return True if conversion successfull (most likely representing ground); false otherwise.
     */
    bool convert_n_to_map_frame(geometry_msgs::Vector3Stamped &n_msg, const bool &last_iteration = true);


    // ---------------------- Callback functions ----------------------
    /** \brief Scan callback function - called for each msg @ hesai pandar topic. Filters + creates subcoud then findes normal vector of ground plane.
     */
    void scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    /** \brief Scan callback function - called for each msg @ plane topic. Increases plane counter.
     */
    void scan_callback_count(const std_msgs::EmptyConstPtr& msg);

public:
    GroundFinder(Preprocessing filtering, Preprocessing subcloud, PlaneSegm plane_alg, bool quiet, bool write2file = false, std::string path = ""){
        // Set private variables
        this->filtering  = filtering;
        this->subcloud   = subcloud;
        this->plane_alg  = plane_alg;
        this->quiet = quiet;
        this->write2file = write2file;
        count_fail = 0;
        plane_counter = 0;

        // Initialize Topics
        pub_subcloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/sub_cloud", 1);
        pub_test     = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/inliers", 1);
        pub_test2    = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/cur_scan_del", 1);
        pub_n   = nh.advertise<geometry_msgs::Vector3Stamped>("/ground_finder/normal_vector", 1);
        pub_vis_n  = nh.advertise<visualization_msgs::Marker>("/ground_finder/normal_marker", 1);

        // Initalize n_marker
        initMarker();

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

        // Subscribe to Plane counter
        sub_p = nh.subscribe("/plane", 1, &GroundFinder::scan_callback_count, this);

        // Subscribe to Hesai laser scanner
        sub_h = nh.subscribe("/lidar/points_undistorted", 200, &GroundFinder::scan_callback, this);
        // sub = nh.subscribe("/hesai/pandar", 1, &GroundFinder::scan_callback, this);

        // Open file stream
        if(write2file){
            csv.open(path);
            // Write header (times)
            csv << "Downsample[ns],BuildFilTree[ns],SearchFil[ns],DeleteFil[ns],BuildTreeSub[ns],SearchSub[ns],DeleteSub[ns],PreProcTotal[ns],Plane[ns],Total[ns],";
            // Wirte header (result = n in map2)
            csv << "nx,ny,nz,planeCount\n";
        }
    }
};

#endif
