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

#include <atomic>
#include <memory>

#include <ground_finder_msgs/ScoredNormalStamped.h>

// Preprocessing types
enum Preprocessing
{
    NONE,
    GEOMETRICAL,
    KD_TREE
};

// Plane segmentation types
enum PlaneSegm
{
    LSF,
    PCA,
    RANSAC,
    RHT,
    RHT2
};

// ----------- Guassian Kernel class for smoothing --------------
class SmoothedGaussian3D
{
public:
    SmoothedGaussian3D(size_t window_size, float sigma, float dt)
        : window_size_(window_size), dt_(dt), kernel_(compute_gaussian_kernel(window_size, sigma, dt)),
          buffers_(3, std::vector<float>(window_size, 0.0f)),
          index_(0)
    {
    }

    // Add new sample and get smoothed value
    std::vector<float> filter(float nx, float ny, float nz)
    {
        // write sample into circ buffer
        size_t write_index = index_.fetch_add(1, std::memory_order_relaxed) % window_size_; // get current index and increment atomically
        buffers_[0][write_index] = nx;
        buffers_[1][write_index] = ny;
        buffers_[2][write_index] = nz;

        std::vector<float> result(3, 0.0f);

        // Compute result (convolution with gaussian kernel)
        for (int axis = 0; axis < 3; ++axis)
        {
            for (size_t k = 0; k < window_size_; ++k)
            {
                size_t buff_index = (write_index + window_size_ - k) % window_size_;
                result[axis] += buffers_[axis][buff_index] * kernel_[k];
            }
        }
        return result;
    }

private:
    size_t window_size_;
    float dt_;
    std::vector<float> kernel_;
    std::vector<std::vector<float>> buffers_; // 3 x circ buffer
    std::atomic<size_t> index_;

    // Compute causal Gaussian kernel (samples at t = 0, -dt, -2dt, ...)
    std::vector<float> compute_gaussian_kernel(size_t N, float sigma, float dt)
    {
        std::vector<float> kernel(N);
        float sum = 0.0f;
        for (size_t i = 0; i < N; ++i)
        {
            float t = -static_cast<float>(i) * dt; // causal: newest sample at index 0
            kernel[i] = std::exp(-(t * t) / (2.0f * sigma * sigma));
            sum += kernel[i];
        }
        if (sum > 0.0f)
        {
            for (auto &n : kernel)
                n /= sum; // normalize since gaussian normalization part is missing in kernel[i]=...
        }
        return kernel;
    }
};

// ---------------------- GroundFinder class ----------------------
class GroundFinder
{
private:
    int count_fail;
    int plane_counter;

    /* Normal calculation */
    std::vector<double> n = {0.0, 0.0, -1.0}; // Normalized normal vector (n) in pandar_frame
    visualization_msgs::Marker n_marker;

    /* ROS variables for node */
    ros::NodeHandle nh;    // Node handle
    ros::Subscriber sub_h; // Subscriber for hesai topic
    ros::Subscriber sub_p; // Subscriber for plane topic
    ros::Subscriber sub_lkf;
    ros::Publisher pub_subcloud; // Publisher of subcloud
    ros::Publisher pub_inliers;
    // ros::Publisher pub_test2;             // TODO take out!
    ros::Publisher pub_n;                 // Publisher of normal vector in map2 frame
    ros::Publisher pub_vis_n;             // Publisher of normal vector marker for rviz
    ros::Publisher pub_smoothed_n;        // Publisher of smoothed normal vector in map2 frame
    ros::Publisher pub_scored_n;          // Publisher of scored normal vector in map2 frame
    ros::Publisher pub_smoothed_scored_n; // Publisher of smoothed scored normal vector in map2 frameSS

    /* TF2 variables */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr}; // Transform listener
    tf2_ros::Buffer tf_buffer;                                        // Transform buffer

    /* Preprocessing steps */
    Preprocessing filtering; // Filter type
    Preprocessing subcloud;  // Subcloud type

    /* Plane segmentation */
    PlaneSegm plane_alg; // Algorithm used for plane segmentation

    /* Writing to file varibales */
    std::string filename;             // Filename of csv
    std::ofstream csv;                // File stream
    std::ofstream scored_normals_log; // File stream for scored normals log
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

    /* EMA smoothing parameters */
    bool enable_normal_smoothing = true;           // Enable smoothing of normal vector
    double normal_smoothing_alpha = 0.1;           // Smoothing factor alpha for normal vector smoothing [0,...,1] higher: more responsive, lower: smoother
    geometry_msgs::Vector3Stamped smoothed_normal; // internally stored smoothed normal (map2 frame)
    bool have_smoothed_normal = false;             // flag if smoothed vector is initialized

    /* Gaussian Kernel Smoothing Parameters */
    std::unique_ptr<SmoothedGaussian3D> gaussian_kernel; // nullptr if not yet created
    double smoothing_cutoff_freq;                        // overridden by rosparam server
    bool use_gaussian_smoothing = false;                 // overrided by rosparam server
    double lidar_rate = 20.0f;                           // taken from rostopic hz /hesai/pandar

    // TODO:tune after final implementation in .cpp
    /* Viewability score variables*/
    bool enable_view_score = false;                                                    // Set to true when first LKF pose is received
    double last_visibility_score = 1.0;                                                // most recent viewability score (1.0 = best, 0.0 = worst)
    size_t min_inliers = 20;                                                           // minimum inliers for plane fitting -> every plane below is too unreliable (?)
    double inlier_scale = 0.3;                                                         // normalization scale: inlier_norm = clamp(inlier_ratio / inlier_scale, 0..1) | choose e.g. 0.5 => 50% inliers means best inlier ratio score (1.0)
    std::deque<ground_finder_msgs::ScoredNormalStamped> scored_normals_sliding_window; // sliding window of scored normals
    static constexpr size_t MAX_WINDOW_SIZE = 40;                                      // for 20Hz normal vecotr rate -> 2s history
    double weight_visibility = 0.6;                                                    // weight for visibility score in combined score calculation
    double weight_inlier_ratio = 0.4;                                                  // weight for inlier ratio score in combined score calculation
    double score_threshold = 0.2;                                                      // normals with score below this threshold are not used but rather fallback value
    double min_score_sliding_window = 0.3;                                             // minimum acceptable score from sliding window
    geometry_msgs::PoseStampedConstPtr last_lkf_pose;                                  // most recent LKF pose
    size_t last_inlier_count = 0;                                                      // #inliers from last plane detection
    size_t last_subcloud_size = 0;                                                     // #points of last filtered subcloud
    double last_roll = 0.0;                                                            // most recent roll angle from LKF pose (radians)
    double last_pitch = 0.0;                                                           // most recent pitch angle from LKF pose (radians)

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
    void scan_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    /** \brief Scan callback function - called for each msg @ plane topic. Increases plane counter.
     */
    void scan_callback_count(const std_msgs::EmptyConstPtr &msg);

    /** \brief LKF pose callback function - called each msg @ LKF topic. Used for orientation-dependent ground view score. */
    void lkf_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // ---------------------- Ground Vector Smoothing ----------------

    geometry_msgs::Vector3Stamped ema_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    geometry_msgs::Vector3Stamped gaussian_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    // ---------------------- Score calculation ----------------------
    /** \brief compute ground visibility [0.1...1] from latest LKF pose and normalized inlier ratio #inlier / #subcloud_points (TODO: currently assumed inliers only contains ground plane)
     *  returns {visibility, inlier_ratio} */
    std::pair<double, double> compute_plane_scores(const geometry_msgs::PoseStampedConstPtr &msg, size_t inlier_count, size_t subcloud_size);

    /**  \brief combine 2 scores into 1 final score */
    double combine_scores(double visibility_score, double inlier_score);

public:
    GroundFinder(Preprocessing filtering, Preprocessing subcloud, PlaneSegm plane_alg, bool quiet, bool write2file = false, std::string path = "")
    {
        // Set private variables
        this->filtering = filtering;
        this->subcloud = subcloud;
        this->plane_alg = plane_alg;
        this->quiet = quiet;
        this->write2file = write2file;
        count_fail = 0;
        plane_counter = 0;

        // Initialize Topics
        pub_subcloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/sub_cloud", 1);
        pub_inliers = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/inliers", 1);
        // pub_test2 = nh.advertise<sensor_msgs::PointCloud2>("/ground_finder/cur_scan_del", 1);
        pub_n = nh.advertise<geometry_msgs::Vector3Stamped>("/ground_finder/normal_vector", 1);
        pub_vis_n = nh.advertise<visualization_msgs::Marker>("/ground_finder/normal_marker", 1);
        pub_smoothed_n = nh.advertise<geometry_msgs::Vector3Stamped>("ground_finder/smoothed_normal_vector", 1);
        pub_scored_n = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("ground_finder/scored_normal_vector", 1);
        pub_smoothed_scored_n = nh.advertise<geometry_msgs::Vector3Stamped>("ground_finder/smoothed_scored_normal_vector", 1);

        // Initialize smoothing parameter
        // read smoothing params from rosparam serve
        ros::NodeHandle pnh("~"); // TODO: check if needed (anonymous node handle for private params)
        pnh.param<bool>("enable_normal_smoothing", enable_normal_smoothing, enable_normal_smoothing);
        pnh.param<double>("normal_smoothing_alpha", normal_smoothing_alpha, normal_smoothing_alpha);
        pnh.param<bool>("use_gaussian_smoothing", use_gaussian_smoothing, use_gaussian_smoothing);
        pnh.param<double>("smoothing_cutoff_freq", smoothing_cutoff_freq, smoothing_cutoff_freq);
        pnh.param<double>("lidar_rate", lidar_rate, lidar_rate);
        pnh.param<double>("weight_visibility", weight_visibility, weight_visibility);
        pnh.param<double>("weight_inlier_ratio", weight_inlier_ratio, weight_inlier_ratio);
        pnh.param<double>("score_threshold", score_threshold, score_threshold);
        pnh.param<double>("min_score_sliding_window", min_score_sliding_window, min_score_sliding_window);

        ROS_INFO("Smoothing params: enable_ema=%s alpha=%.3f use_gauss=%s cutoff=%.3f lidar_rate=%.3f",
                 enable_normal_smoothing ? "true" : "false", normal_smoothing_alpha,
                 use_gaussian_smoothing ? "true" : "false", smoothing_cutoff_freq, lidar_rate);

        ROS_INFO("Score thresholds: current_threshold=%.3f, min_score_sliding_window=%.3f",
                 score_threshold, min_score_sliding_window);

        // Initialize Gaussian Kernel Smoothing if enabled
        if (use_gaussian_smoothing == true)
        {
            double sigma = 1.0 / (2.0 * M_PI * smoothing_cutoff_freq); // 1/(1/s) = s
            double dt = 1.0 / lidar_rate;                              // lidar rate = 20 // 1 / (1/s) = s
            int win_size = 6 * sigma / dt;                             // unitless -- 19 for cutoff_freq of 1 Hz -- 39 FOR 0.5 Hz
            win_size = win_size % 2 == 0 ? win_size + 1 : win_size;
            gaussian_kernel.reset(new SmoothedGaussian3D(win_size, sigma, dt)); // create kernel

            ROS_INFO("created gaussian kernel smoothing with \n window size: %d, sigma: %.4f s, dt: %.4f s, cutoff freq: %.4f Hz, lidar_rate: %.4f Hz", win_size, sigma, dt, smoothing_cutoff_freq, lidar_rate);
        }

        // Initalize n_marker
        initMarker();

        // Wait for first transform (pandar_frame to map2)
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
        ROS_WARN("Waiting for TF Listener!");
        bool tf_listener_fail = true;
        while (tf_listener_fail && ros::ok())
        {
            try
            {
                tf_listener_fail = false;
                geometry_msgs::TransformStamped t = tf_buffer.lookupTransform("map2", "pandar_frame", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
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

        sub_lkf = nh.subscribe("/lkf/pose", 1, &GroundFinder::lkf_pose_callback, this);

        // Open file stream
        if (write2file)
        {
            csv.open(path);
            // Write header (times)
            csv << "Downsample[ns],BuildFilTree[ns],SearchFil[ns],DeleteFil[ns],BuildTreeSub[ns],SearchSub[ns],DeleteSub[ns],PreProcTotal[ns],Plane[ns],Total[ns],";
            // Wirte header (result = n in map2)
            csv << "nx,ny,nz,planeCount\n";
        }

        // Always write scored normals log
        // trunc to overwrite existing file
        std::string scored_normals_path;
        if (!path.empty() && path.find_last_of('/') != std::string::npos)
        {
            scored_normals_path = path.substr(0, path.find_last_of('/')) + "/scored_normals.csv";
        }
        else
        {
            // Default path when no file is specified
            const char *home_dir = std::getenv("HOME");
            std::string default_dir = home_dir ? std::string(home_dir) + "/catkin_ws/src/ground_finder/data/" : "/tmp/";
            scored_normals_path = default_dir + "scored_normals.csv";
        }
        scored_normals_log.open(scored_normals_path, std::ios::out | std::ios::trunc);
        scored_normals_log << "timestamp,nx,ny,nz,roll,pitch,pub_vis_score,pub_inlier_score,pub_combined_score,curr_vis_score,curr_inlier_score,curr_combined_score,inlier_count,subcloud_size,inlier_ratio,using_fallback,fallback_unavailable\n";
        ROS_INFO("Scored normals log: %s", scored_normals_path.c_str());
    }
};

#endif
