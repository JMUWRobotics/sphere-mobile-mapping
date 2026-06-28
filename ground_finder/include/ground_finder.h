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
#include <ros/package.h>
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
#include <mutex>
#include <memory>
#include <ctime>

#include <ground_finder_msgs/ScoredNormalStamped.h>
#include <state_estimator_msgs/Estimator.h>

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

    // Export kernel to CSV for plotting
    void exportKernelToCSV(const std::string &filepath) const
    {
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open kernel export file: %s", filepath.c_str());
            return;
        }

        file << "index,time_s,weight\n";
        for (size_t i = 0; i < window_size_; ++i)
        {
            float t = -static_cast<float>(i) * dt_;
            file << i << "," << t << "," << kernel_[i] << "\n";
        }
        file.close();
        ROS_INFO("Gaussian kernel exported to: %s", filepath.c_str());
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
    ros::Subscriber sub_lio;
    ros::Publisher pub_subcloud; // Publisher of subcloud
    ros::Publisher pub_inliers;
    // ros::Publisher pub_test2;                 // TODO take out!
    ros::Publisher pub_n;                        // Publisher of normal vector in map_lio frame
    ros::Publisher pub_vis_n;                    // Publisher of normal vector marker for rviz
    ros::Publisher pub_smoothed_n;               // Publisher of smoothed normal vector in map_lio frame
    ros::Publisher pub_scored_n;                 // Publisher of scored normal vector in map_lio frame
    ros::Publisher pub_smoothed_scored_n;        // Publisher of smoothed scored normal vector in map_lio frame
    ros::Publisher pub_scored_n_pandar;          // Publisher of scored normal vector in pandar frame
    ros::Publisher pub_smoothed_scored_n_pandar; // Publisher of smoothed scored normal vector in pandar frame

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

    // Timing CSV logging (separate from gf_file profiling CSV)
    bool timing_csv_enabled_ = false;
    std::string timing_csv_path_;
    std::ofstream timing_csv_file_;
    std::mutex timing_csv_mutex_;

    /* Output regulation */
    bool quiet;

    /* Constants */
    const float radius_sphere = 0.145; // radius sphere used for query point calculation
    const float radius_filter = 0.35;  // radius sphere + noise + hands = 0.35 m (from rviz) used for filtering
    const float radius_subcloud = 2.0; // radius used for geometrical subcloud
    const float height_subcloud = 0.2; // height used for geometrical subcloud
    const float ds_size = 0.10;        // leaf size used for downsampling after filtering
    const int k = 150;                 // k used for kNN for kd tree subcloud //TODO: 150 = for ransac; 50 = better for rht
    const int max_iterations_plane_detection = 3;

    /* EMA smoothing parameters */
    bool enable_normal_smoothing = true;           // Enable smoothing of normal vector
    double normal_smoothing_alpha = 0.1;           // Smoothing factor alpha for normal vector smoothing [0,...,1] higher: more responsive, lower: smoother
    geometry_msgs::Vector3Stamped smoothed_normal; // internally stored smoothed normal (map_lio frame)
    bool have_smoothed_normal = false;             // flag if smoothed vector is initialized

    /* Gaussian Kernel Smoothing Parameters */
    std::unique_ptr<SmoothedGaussian3D> gaussian_kernel; // nullptr if not yet created
    double smoothing_cutoff_freq;                        // overridden by rosparam server
    bool use_gaussian_smoothing = false;                 // overrided by rosparam server
    double lidar_rate = 20.0f;                           // taken from rostopic hz /hesai/pandar

    // TODO:tune after final implementation in .cpp
    /* Viewability score variables*/
    bool enable_view_score = false;                                                    // Set to true when first lio pose is received
    bool enable_scoring = true;                                                        // Enable/disable scoring and fallback mechanism (ROS param)
    double last_visibility_score = 1.0;                                                // most recent viewability score (1.0 = best, 0.0 = worst)
    size_t min_inliers = 20;                                                           // minimum inliers for plane fitting -> every plane below is too unreliable (?)
    double inlier_scale = 0.3;                                                         // normalization scale: inlier_norm = clamp(inlier_ratio / inlier_scale, 0..1) | choose e.g. 0.5 => 50% inliers means best inlier ratio score (1.0)
    std::deque<ground_finder_msgs::ScoredNormalStamped> scored_normals_sliding_window; // sliding window of scored normals
    static constexpr size_t MAX_WINDOW_SIZE = 40;                                      // for 20Hz normal vecotr rate -> 2s history
    double weight_visibility = 0.6;                                                    // weight for visibility score in combined score calculation
    double weight_inlier_ratio = 0.4;                                                  // weight for inlier ratio score in combined score calculation
    double score_threshold = 0.2;                                                      // normals with score below this threshold are not used but rather fallback value
    double min_score_sliding_window = 0.3;                                             // minimum acceptable score from sliding window

    // Validation parameters (GGF-style)
    bool enable_eigenvalue_validation = false;  // Enable combined eigenvalue + eigenvector check
    double eigenvalue_ratio_threshold = 0.1;    // Threshold for λ3/(λ1+λ2)
    double max_eigenvector_z_component = 0.1;   // Max z-component of dominant eigenvectors
    bool enable_plane_angle_validation = true;  // Enable angle-based wall rejection
    double wall_threshold = 0.707;              // Threshold for |dot(normal,down)|; cos(45°)=0.707
    bool enable_z_mean_validation = false;      // Enable Z-mean deviation check
    double max_z_deviation = 0.2;               // Max deviation from robot Z [m]
    bool enable_convex_hull_validation = false; // Enable convex hull center distance check
    double max_hull_distance = 1.0;             // Max 3D distance from robot to hull center [m]

    state_estimator_msgs::EstimatorConstPtr last_lio_pose; // most recent lio pose
    size_t last_inlier_count = 0;                          // #inliers from last plane detection
    size_t last_subcloud_size = 0;                         // #points of last filtered subcloud
    double last_roll = 0.0;                                // most recent roll angle from lio pose (radians)
    double last_pitch = 0.0;                               // most recent pitch angle from lio pose (radians)

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

    /** \brief Converts the normal vector n from pandar_frame to the map_lio frame, while ensuring that n always points into ground.
     * \param[out] n_msg n_msg The normal vector message transformed into the map_lio frame
     * \return True if conversion successfull (most likely representing ground); false otherwise.
     */
    bool convert_n_to_map_frame(geometry_msgs::Vector3Stamped &n_msg, const bool &last_iteration = true);

    /** \brief Validate that the current plane represents ground
     * \param[in,out] normal Normal vector to check
     * \param[in] inlier_cloud Point cloud used for validation checks
     * \param[in] robot_z Robot-center Z coordinate
     * \param[in] lambda1 Largest eigenvalue from PCA
     * \param[in] lambda2 Middle eigenvalue from PCA
     * \param[in] lambda3 Smallest eigenvalue from PCA
     * \param[in] v1_z Z-component of eigenvector for lambda1
     * \param[in] v2_z Z-component of eigenvector for lambda2
     * \return true if valid ground plane, false otherwise
     */
    bool validateGroundNormal(std::vector<double> &normal,
                              const pcl::PointCloud<PointType>::Ptr &inlier_cloud,
                              double robot_z,
                              float lambda1 = 0.0f,
                              float lambda2 = 0.0f,
                              float lambda3 = 0.0f,
                              float v1_z = 0.0f,
                              float v2_z = 0.0f);

    // ---------------------- Callback functions ----------------------
    /** \brief Scan callback function - called for each msg @ hesai pandar topic. Filters + creates subcoud then findes normal vector of ground plane.
     */
    void scan_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    /** \brief Scan callback function - called for each msg @ plane topic. Increases plane counter.
     */
    void scan_callback_count(const std_msgs::EmptyConstPtr &msg);

    /** \brief lio pose callback function - called each msg @ lio topic. Used for orientation-dependent ground view score. */
    void lio_pose_callback(const state_estimator_msgs::EstimatorConstPtr &msg);

    // ---------------------- Ground Vector Smoothing ----------------

    geometry_msgs::Vector3Stamped ema_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    geometry_msgs::Vector3Stamped gaussian_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    // ---------------------- Score calculation ----------------------
    /** \brief compute ground visibility [0.1...1] from latest lio pose and normalized inlier ratio #inlier / #subcloud_points (TODO: currently assumed inliers only contains ground plane)
     *  returns {visibility, inlier_ratio} */
    std::pair<double, double> compute_plane_scores(const state_estimator_msgs::EstimatorConstPtr &msg, size_t inlier_count, size_t subcloud_size);

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
        pub_smoothed_scored_n = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("ground_finder/smoothed_scored_normal_vector", 1);
        pub_scored_n_pandar = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("ground_finder/scored_normal_vector_pandar", 1);
        pub_smoothed_scored_n_pandar = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("ground_finder/smoothed_scored_normal_vector_pandar", 1);

        // Initialize smoothing parameter
        // read smoothing params from rosparam serve
        ros::NodeHandle pnh("~"); // TODO: check if needed (anonymous node handle for private params)
        pnh.param<bool>("gf_enable_normal_smoothing", enable_normal_smoothing, enable_normal_smoothing);
        pnh.param<double>("gf_normal_smoothing_alpha", normal_smoothing_alpha, normal_smoothing_alpha);
        pnh.param<bool>("gf_use_gaussian_smoothing", use_gaussian_smoothing, use_gaussian_smoothing);
        pnh.param<double>("gf_smoothing_cutoff_freq", smoothing_cutoff_freq, smoothing_cutoff_freq);
        pnh.param<double>("gf_lidar_rate", lidar_rate, lidar_rate);
        pnh.param<bool>("gf_enable_scoring", enable_scoring, enable_scoring);
        pnh.param<double>("gf_weight_visibility", weight_visibility, weight_visibility);
        pnh.param<double>("gf_weight_inlier_ratio", weight_inlier_ratio, weight_inlier_ratio);
        pnh.param<double>("gf_score_threshold", score_threshold, score_threshold);
        pnh.param<double>("gf_min_score_sliding_window", min_score_sliding_window, min_score_sliding_window);
        pnh.param<bool>("gf_enable_eigenvalue_validation", enable_eigenvalue_validation, enable_eigenvalue_validation);
        pnh.param<double>("gf_eigenvalue_ratio_threshold", eigenvalue_ratio_threshold, eigenvalue_ratio_threshold);
        pnh.param<double>("gf_max_eigenvector_z_component", max_eigenvector_z_component, max_eigenvector_z_component);
        pnh.param<bool>("gf_enable_plane_angle_validation", enable_plane_angle_validation, enable_plane_angle_validation);
        pnh.param<double>("gf_wall_threshold", wall_threshold, wall_threshold);
        pnh.param<bool>("gf_enable_z_mean_validation", enable_z_mean_validation, enable_z_mean_validation);
        pnh.param<double>("gf_max_z_deviation", max_z_deviation, max_z_deviation);
        pnh.param<bool>("gf_enable_convex_hull_validation", enable_convex_hull_validation, enable_convex_hull_validation);
        pnh.param<double>("gf_max_hull_distance", max_hull_distance, max_hull_distance);
        pnh.param<bool>("gf_timing_csv_enabled", timing_csv_enabled_, true);
        pnh.param<std::string>("gf_timing_csv_file", timing_csv_path_, std::string(""));
        if (timing_csv_path_.empty())
        {
            timing_csv_path_ = ros::package::getPath("ground_finder") + "/data/timings.csv";
        }

        ROS_INFO("[GF] Smoothing params: enable_ema=%s alpha=%.3f use_gauss=%s cutoff=%.3f lidar_rate=%.3f",
                 enable_normal_smoothing ? "true" : "false", normal_smoothing_alpha,
                 use_gaussian_smoothing ? "true" : "false", smoothing_cutoff_freq, lidar_rate);

        ROS_INFO("[GF] Scoring and fallback: %s", enable_scoring ? "ENABLED" : "DISABLED");
        if (enable_scoring)
        {
            ROS_INFO("[GF] Score thresholds: current_threshold=%.3f, min_score_sliding_window=%.3f",
                     score_threshold, min_score_sliding_window);
        }

        ROS_INFO("[GF] Validation: angle=%s eigen=%s z_mean=%s hull=%s",
                 enable_plane_angle_validation ? "enabled" : "disabled",
                 enable_eigenvalue_validation ? "enabled" : "disabled",
                 enable_z_mean_validation ? "enabled" : "disabled",
                 enable_convex_hull_validation ? "enabled" : "disabled");
        ROS_INFO("[GF] Validation thresholds: wall=%.3f eigen_ratio=%.3f max_dom_z=%.3f max_z_dev=%.3f max_hull_dist=%.3f",
                 wall_threshold, eigenvalue_ratio_threshold, max_eigenvector_z_component, max_z_deviation, max_hull_distance);

        if (timing_csv_enabled_ && !timing_csv_path_.empty())
        {
            std::string algo_name = "RANSAC";
            switch (plane_alg)
            {
            case LSF:
                algo_name = "LSF";
                break;
            case PCA:
                algo_name = "PCA";
                break;
            case RANSAC:
                algo_name = "RANSAC";
                break;
            case RHT:
                algo_name = "RHT";
                break;
            case RHT2:
                algo_name = "RHT2";
                break;
            }

            std::time_t now = std::time(nullptr);
            std::tm now_tm;
            localtime_r(&now, &now_tm);

            char timestamp[32];
            std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &now_tm);

            const std::string suffix = std::string("_") + algo_name + "_" + timestamp;
            const std::size_t slash_pos = timing_csv_path_.find_last_of('/');
            const std::size_t dot_pos = timing_csv_path_.find_last_of('.');

            if (dot_pos != std::string::npos &&
                (slash_pos == std::string::npos || dot_pos > slash_pos))
            {
                timing_csv_path_.insert(dot_pos, suffix);
            }
            else
            {
                timing_csv_path_ += suffix + ".csv";
            }

            timing_csv_file_.open(timing_csv_path_, std::ios::out | std::ios::app);
            if (timing_csv_file_.is_open())
            {
                timing_csv_file_.seekp(0, std::ios::end);
                if (timing_csv_file_.tellp() == 0)
                {
                    timing_csv_file_ << "timestamp,preprocessing_ms,plane_ms,total_ms,success\n";
                    timing_csv_file_.flush();
                }
                ROS_INFO("[GF] Timing CSV logging enabled: %s", timing_csv_path_.c_str());
            }
            else
            {
                ROS_ERROR("[GF] Failed to open timing CSV file: %s", timing_csv_path_.c_str());
                timing_csv_enabled_ = false;
            }
        }

        // Initialize Gaussian Kernel Smoothing if enabled
        if (use_gaussian_smoothing == true)
        {
            double sigma = (sqrt(2.0 * log(2))) / (2.0 * M_PI * smoothing_cutoff_freq); // 1.0 / (2.0 * M_PI * smoothing_cutoff_freq); // 1/(1/s) = s // maybe change to (ln(2) / (2 * M_PI * cutoff_freq)) to get -3dB cutoff point?
            double dt = 1.0 / lidar_rate;                                               // lidar rate = 20 // 1 / (1/s) = s
            int win_size = 6.0f * sigma / dt;                                           // unitless -- 19 for cutoff_freq of 1 Hz -- 39 FOR 0.5 Hz
            win_size = win_size % 2 == 0 ? win_size + 1 : win_size;
            gaussian_kernel.reset(new SmoothedGaussian3D(win_size, sigma, dt)); // create kernel

            ROS_INFO("[GF] created gaussian kernel smoothing with \n window size: %d, sigma: %.4f s, dt: %.4f s, cutoff freq: %.4f Hz, lidar_rate: %.4f Hz", win_size, sigma, dt, smoothing_cutoff_freq, lidar_rate);

            // Export kernel for plotting
            gaussian_kernel->exportKernelToCSV("/tmp/gaussian_kernel.csv");
        }

        // Initalize n_marker
        initMarker();

        // Wait for first transform (pandar_frame to map_lio)
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);
        ROS_WARN("[GF] Waiting for TF Listener!");
        bool tf_listener_fail = true;
        while (tf_listener_fail && ros::ok())
        {
            try
            {
                tf_listener_fail = false;
                geometry_msgs::TransformStamped t = tf_buffer.lookupTransform("map_lio", "pandar_frame", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                if (!quiet)
                {
                    ROS_WARN_THROTTLE(1.0, "[GF] tf lookup failing... reason: %s", ex.what());
                }
                tf_listener_fail = true;
                ros::Duration(0.1).sleep();
            }
        }
        ROS_INFO("[GF] Received Transformation! Starting subscriber now!\n");

        // Subscribe to Plane counter
        sub_p = nh.subscribe("/plane", 1, &GroundFinder::scan_callback_count, this);

        // Subscribe to Hesai laser scanner
        sub_h = nh.subscribe("/lidar/points_undistorted", 200, &GroundFinder::scan_callback, this);
        // sub = nh.subscribe("/hesai/pandar", 1, &GroundFinder::scan_callback, this);

        sub_lio = nh.subscribe("/all_pose_out", 1, &GroundFinder::lio_pose_callback, this);

        // Open file stream
        if (write2file)
        {
            csv.open(path);
            // Write header (times)
            csv << "Downsample[ns],BuildFilTree[ns],SearchFil[ns],DeleteFil[ns],BuildTreeSub[ns],SearchSub[ns],DeleteSub[ns],PreProcTotal[ns],Plane[ns],Total[ns],";
            // Wirte header (result = n in map_lio)
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
        ROS_INFO("[GF] Scored normals log: %s", scored_normals_path.c_str());
    }
};

#endif
