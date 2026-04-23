/*
 * calculation of ground plane's normal vector from global map
 * at current pose location
 *
 * Based on ground_finder.h by Carolin Bösch
 * Adapted for map-based operation
 */

#ifndef GLOBAL_GROUND_FINDER_H
#define GLOBAL_GROUND_FINDER_H

#include <chrono>
#include <deque>
#include <memory>
#include <chrono>
#include <mutex>
#include <cstdint>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <state_estimator_msgs/Estimator.h>
#include <visualization_msgs/Marker.h>
#include <ground_finder_msgs/ScoredNormalStamped.h>

#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/common.h>
#include <atomic>

#include "hough.h"
#include "math_helpers.h"
#include <lio_gf_nodelet_manager/shared_ikdtree_store.h>

typedef pcl::PointXYZ PointType;

enum PlaneSegm
{
    PCA,
    RANSAC,
    RHT,
    RHT2
};

// ----------- Gaussian Kernel class for smoothing --------------
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
            float t = -static_cast<float>(i) * dt; // newest sample at index 0
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

// ---------------------- GlobalGroundFinder class ----------------------
class GlobalGroundFinder
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber sub_map;     // Global map from /map_out
    ros::Subscriber sub_trigger; // Trigger processing when a registered pose path is published by lio node
    ros::Subscriber sub_pose;    // Optional fallback pose source

    ros::Publisher pub_local_cloud;              // Local extracted region for visualization
    ros::Publisher pub_inliers;                  // Inlier points used for plane fit
    ros::Publisher pub_rejected_inliers;         // Inlier points from rejected intermediate plane candidates
    ros::Publisher pub_n;                        // Raw normal vector
    ros::Publisher pub_scored_n;                 // Scored normal with quality metrics
    ros::Publisher pub_smoothed_n;               // Smoothed raw normal
    ros::Publisher pub_smoothed_scored_n;        // Smoothed scored normal
    ros::Publisher pub_scored_n_pandar;          // Scored normal transformed to pandar_frame
    ros::Publisher pub_smoothed_scored_n_pandar; // Smoothed & scored normal transformed to pandar_frame
    ros::Publisher pub_n_marker;                 // Visualization marker
    ros::Publisher pub_hull_center;              // Convex hull center point visualization
    ros::Publisher pub_shared_map_debug;         // Debug map reconstructed from shared IKD-tree

    boost::shared_ptr<pcl::PointCloud<PointType>> global_map_;
    boost::shared_ptr<pcl::KdTreeFLANN<PointType>> kdtree_;
    bool map_received_;
    ros::Time map_timestamp_;
    uint64_t last_snapshot_version_;
    std::string trigger_topic_;
    bool publish_shared_map_debug_;
    bool debug_publish_;
    std::string shared_map_debug_topic_;

    geometry_msgs::PoseStamped current_pose_; // latest pose
    bool pose_received_;
    std::mutex pose_mutex_;       // Protects current_pose_ during raeading / writing
    std::mutex processing_mutex_; // ensures only one processAtCurrentPose() runs at a time

    std::vector<double> n_; // Current normal estimate [nx, ny, nz]
    visualization_msgs::Marker normal_marker_;

    std::deque<ground_finder_msgs::ScoredNormalStamped> scored_window_;
    static constexpr size_t MAX_WINDOW_SIZE = 40; // 2s history at 20Hz
    size_t last_inlier_count_;
    size_t last_local_cloud_size_;
    double last_roll_;
    double last_pitch_;

    // Params (set via launch)
    PlaneSegm plane_algorithm_;
    bool quiet_;
    bool enable_scoring_;
    double extraction_radius_;           // Radius for local extraction [m]
    double extraction_height_;           // Height limit for local extraction [m]
    double min_points_for_plane_;        // Minimum points needed for plane fitting
    double wall_threshold_;              // cos(angle) threshold to reject walls
    double score_threshold_;             // Minimum score to accept current normal
    double min_score_window_;            // Minimum score from history window
    double weight_visibility_;           // Weight for visibility score
    double weight_inlier_ratio_;         // Weight for inlier ratio score
    double min_inliers_;                 // Minimum inliers for valid plane
    double inlier_scale_;                // Normalization scale for inlier ratio
    int max_iterations_plane_detection_; // Max iterations for RANSAC wall rejection

    // Point distribution validation (improved wall rejection)
    bool enable_eigenvalue_validation_;  // Enable combined eigenvalue + eigenvector check (planarity + xy-plane dominance)
    double eigenvalue_ratio_threshold_;  // Threshold for λ3/(λ1+λ2) to detect non-planar (wall)
    double max_eigenvector_z_component_; // Max z-component of dominant eigenvectors (v1, v2) to ensure xy-plane spread
    bool enable_plane_angle_validation_; // Enable angle-based validation (wall threshold check)
    bool enable_z_mean_validation_;      // Enable Z-mean deviation check
    double max_z_deviation_;             // Maximum allowed deviation of Z-mean from robot Z [m]
    bool enable_convex_hull_validation_; // Enable convex hull center distance check (tunnel mode)
    double max_hull_distance_;           // Maximum 3D distance from robot to hull center [m]

    int count_success_;
    int count_fail_;
    int count_valid_planes_;   // Number of planes that passed validation
    int count_invalid_planes_; // Number of planes that failed validation

    bool write2file;
    std::string log_file_path_;
    std::ofstream log_file_;

    bool enable_normal_smoothing_;                        // Enable smoothing of normal vector
    double normal_smoothing_alpha_;                       // EMA smoothing factor [0,1]
    bool use_gaussian_smoothing_;                         // Use Gaussian kernel instead of EMA
    double smoothing_cutoff_freq_;                        // Cutoff frequency for Gaussian kernel
    double update_rate_;                                  // Expected update rate (Hz)
    std::unique_ptr<SmoothedGaussian3D> gaussian_kernel_; // Gaussian kernel filter
    geometry_msgs::Vector3Stamped smoothed_normal_;       // Internally stored smoothed normal
    bool have_smoothed_normal_;                           // Flag if smoothed vector is initialized

    // TF2 variables for frame transforms of published normals
    tf2_ros::Buffer tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

    // ---------------------- Callback functions ----------------------
    /** \brief Callback for global map updates
     * \param[in] msg PointCloud2 message containing global map
     */
    void mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    /** \brief Callback for registered pose trigger
     * \param[in] msg Estimator message on trigger topic. The embedded pose is used.
     */
    void triggerCallback(const state_estimator_msgs::EstimatorConstPtr &msg);

    /** \brief Callback for pose updates
     * \param[in] msg PoseStamped message with current robot pose
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // ---------------------- Normal computation functions ----------------------
    /** \brief Process current pose: extract local region and compute normal
     */
    void processAtCurrentPose();

    /** \brief Publish full map from shared IKD-tree for debugging
     * \param[in] trigger_stamp Fallback timestamp when shared handle stamp is zero
     */
    void publishSharedMapDebug(const ros::Time &trigger_stamp);

    /** \brief Publish inlier cloud for rejected intermediate candidates
     * \param[in] rejected_cloud Candidate inliers that were rejected
     */
    void publishRejectedInliers(const pcl::PointCloud<PointType>::Ptr &rejected_cloud);

    /** \brief Extract local point cloud around current pose
     * \param[in] pose Current pose in map_lkf(?) frame
     * \param[out] local_cloud Extracted local point cloud
     * \return true if successful, false if insufficient points
     */
    bool extractLocalCloud(const geometry_msgs::PoseStamped &pose,
                           pcl::PointCloud<PointType>::Ptr &local_cloud);

    /** \brief Fit ground plane to local cloud
     * \param[in] local_cloud Point cloud to fit
     * \param[out] normal Output normal vector [nx, ny, nz]
     * \param[out] inlier_count Number of inliers
     * \param[out] inlier_cloud Point cloud containing inliers used for the final normal
     * \return true if successful plane found, false otherwise
     */
    bool fitGroundPlane(const pcl::PointCloud<PointType>::Ptr &local_cloud,
                        std::vector<double> &normal,
                        size_t &inlier_count,
                        pcl::PointCloud<PointType>::Ptr &inlier_cloud);

    /** \brief Fit plane using PCA
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_cloud Point cloud used to compute the normal
     * \return true if successful
     */
    bool fitPlanePCA(const pcl::PointCloud<PointType>::Ptr &cloud,
                     std::vector<double> &normal,
                     pcl::PointCloud<PointType>::Ptr &inlier_cloud);

    /** \brief Fit plane using RANSAC
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \param[out] inlier_cloud Inlier points used for the final normal
     * \return true if successful
     */
    bool fitPlaneRANSAC(const pcl::PointCloud<PointType>::Ptr &cloud,
                        std::vector<double> &normal,
                        size_t &inlier_count,
                        pcl::PointCloud<PointType>::Ptr &inlier_cloud);

    /** \brief Fit plane using RHT (Randomized Hough Transform)
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \param[out] inlier_cloud Inlier points used for the final normal
     * \return true if successful
     */
    bool fitPlaneRHT(const pcl::PointCloud<PointType>::Ptr &cloud,
                     std::vector<double> &normal,
                     size_t &inlier_count,
                     pcl::PointCloud<PointType>::Ptr &inlier_cloud);

    /** \brief Fit plane using RHT2 (RHT + PCA refinement)
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \param[out] inlier_cloud Inlier points used for the final normal
     * \return true if successful
     */
    bool fitPlaneRHT2(const pcl::PointCloud<PointType>::Ptr &cloud,
                      std::vector<double> &normal,
                      size_t &inlier_count,
                      pcl::PointCloud<PointType>::Ptr &inlier_cloud);

    /** \brief Validate that normal represents ground (not wall/ceiling)
     * Performs multi-layer validation: angle check (wall rejection), combined eigenvalue+eigenvector check
     * (planarity + xy-plane dominance), and Z-mean check (height deviation from robot center).
     * \param[in,out] normal Normal vector to check
     * \param[in] inlier_cloud Point cloud of inliers (for Z-mean validation)
     * \param[in] robot_z Robot-center Z coordinate (for Z-mean validation)
     * \param[in] lambda1 Largest eigenvalue from PCA (0.0 = skip combined check)
     * \param[in] lambda2 Middle eigenvalue from PCA (0.0 = skip combined check)
     * \param[in] lambda3 Smallest eigenvalue from PCA (0.0 = skip combined check)
     * \param[in] v1_z Z-component of eigenvector for lambda1 (0.0 = skip combined check)
     * \param[in] v2_z Z-component of eigenvector for lambda2 (0.0 = skip combined check)
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

    /** \brief Compute quality scores for current normal
     * \param[in] pose Current pose
     * \param[in] inlier_count Number of inliers
     * \param[in] local_size Total points in local region
     * \return pair<visibility_score, inlier_score>
     */
    std::pair<double, double> compute_scores(const geometry_msgs::PoseStamped &pose,
                                             size_t inlier_count,
                                             size_t local_size);

    /** \brief Combine individual scores into overall quality metric
     * \param[in] vis_score Visibility score [0,1]
     * \param[in] inlier_score Inlier ratio score [0,1]
     * \return Combined score [0,1]
     */
    double combine_scores(double vis_score, double inlier_score);

    /** \brief Check sliding window for better historical normal
     * \param[in] current_score Score of current normal
     * \param[out] fallback_normal Best normal from history
     * \param[out] fallback_msg Full scored message for fallback
     * \return true if suitable fallback found
     */
    bool find_fallback_normal(double current_score,
                              std::vector<double> &fallback_normal,
                              ground_finder_msgs::ScoredNormalStamped &fallback_msg);

    // ---------------------- Smoothing functions ----------------------
    /** \brief Apply exponential moving average smoothing to normal vector
     * \param[in] ground_vector Input normal vector to smooth
     * \return Smoothed normal vector
     */
    geometry_msgs::Vector3Stamped ema_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    /** \brief Apply Gaussian kernel smoothing to normal vector
     * \param[in] ground_vector Input normal vector to smooth
     * \return Smoothed normal vector
     */
    geometry_msgs::Vector3Stamped gaussian_smoothing(const geometry_msgs::Vector3Stamped &ground_vector);

    // ---------------------- Helper functions ----------------------
    /** \brief Initialize visualization marker
     */
    void initMarker();

    /** \brief Publish visualization marker for normal vector
     * \param[in] normal Normal vector to visualize
     * \param[in] stamp Timestamp for marker
     */
    void publish_normal_marker(const std::vector<double> &normal, const ros::Time &stamp);
    void publishHullCenterMarker(const geometry_msgs::Point &hull_center);

    /** \brief Write computed ground normal vector to CSV file
     * \param[in] stamp Timestamp
     * \param[in] normal Normal vector
     * \param[in] query_pose Query pose used for extraction
     * \param[in] pub_vis_score Published visibility score
     * \param[in] pub_inlier_score Published inlier score
     * \param[in] pub_combined_score Published combined score
     * \param[in] curr_vis_score Current visibility score before fallback
     * \param[in] curr_inlier_score Current inlier score before fallback
     * \param[in] curr_combined_score Current combined score before fallback
     * \param[in] inlier_count Number of inliers for accepted plane
     * \param[in] subcloud_size Local cloud size used for fitting
     * \param[in] using_fallback Whether a fallback normal was published
     */
    void log_results(const ros::Time &stamp,
                     const std::vector<double> &normal,
                     const geometry_msgs::Pose &query_pose,
                     double pub_vis_score,
                     double pub_inlier_score,
                     double pub_combined_score,
                     double curr_vis_score,
                     double curr_inlier_score,
                     double curr_combined_score,
                     size_t inlier_count,
                     size_t subcloud_size,
                     bool using_fallback);

public:
    /** \brief Constructor
     * \param[in] nh ROS node handle
     * \param[in] pnh Private node handle
     * \param[in] plane_algorithm Plane fitting algorithm to use
     */
    GlobalGroundFinder(ros::NodeHandle &nh, ros::NodeHandle &pnh, PlaneSegm plane_algorithm);

    /** \brief Destructor
     */
    ~GlobalGroundFinder();
};

#endif // GLOBAL_GROUND_FINDER_H
