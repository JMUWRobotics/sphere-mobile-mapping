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

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <ground_finder_msgs/ScoredNormalStamped.h>

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

    ros::Subscriber sub_map;  // Global map from /map_out
    ros::Subscriber sub_pose; // Current pose

    ros::Publisher pub_local_cloud;            // Local extracted region for visualization
    ros::Publisher pub_inliers;                // Inlier points used for plane fit
    ros::Publisher pub_normal;                 // Raw normal vector
    ros::Publisher pub_scored_normal;          // Scored normal with quality metrics
    ros::Publisher pub_smoothed_normal;        // Smoothed raw normal
    ros::Publisher pub_smoothed_scored_normal; // Smoothed scored normal
    ros::Publisher pub_normal_marker;          // Visualization marker

    boost::shared_ptr<pcl::PointCloud<PointType>> global_map_;
    boost::shared_ptr<pcl::KdTreeFLANN<PointType>> kdtree_;
    bool map_received_;
    ros::Time map_timestamp_;

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

    int count_success_;
    int count_fail_;

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

    // ---------------------- Callback functions ----------------------
    /** \brief Callback for global map updates
     * \param[in] msg PointCloud2 message containing global map
     */
    void mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    /** \brief Callback for pose updates
     * \param[in] msg PoseStamped message with current robot pose
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // ---------------------- Normal computation functions ----------------------
    /** \brief Process current pose: extract local region and compute normal
     */
    void processAtCurrentPose();

    /** \brief Extract local point cloud around current pose
     * \param[in] pose Current pose in map frame
     * \param[out] local_cloud Extracted local point cloud
     * \return true if successful, false if insufficient points
     */
    bool extractLocalCloud(const geometry_msgs::PoseStamped &pose,
                           pcl::PointCloud<PointType>::Ptr &local_cloud);

    /** \brief Fit ground plane to local cloud
     * \param[in] local_cloud Point cloud to fit
     * \param[out] normal Output normal vector [nx, ny, nz]
     * \param[out] inlier_count Number of inliers
     * \return true if successful plane found, false otherwise
     */
    bool fitGroundPlane(const pcl::PointCloud<PointType>::Ptr &local_cloud,
                        std::vector<double> &normal,
                        size_t &inlier_count);

    /** \brief Fit plane using PCA
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \return true if successful
     */
    bool fitPlanePCA(const pcl::PointCloud<PointType>::Ptr &cloud,
                     std::vector<double> &normal);

    /** \brief Fit plane using RANSAC
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \return true if successful
     */
    bool fitPlaneRANSAC(const pcl::PointCloud<PointType>::Ptr &cloud,
                        std::vector<double> &normal,
                        size_t &inlier_count);

    /** \brief Fit plane using RHT (Randomized Hough Transform)
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \return true if successful
     */
    bool fitPlaneRHT(const pcl::PointCloud<PointType>::Ptr &cloud,
                     std::vector<double> &normal,
                     size_t &inlier_count);

    /** \brief Fit plane using RHT2 (RHT + PCA refinement)
     * \param[in] cloud Input point cloud
     * \param[out] normal Output normal vector
     * \param[out] inlier_count Number of inliers
     * \return true if successful
     */
    bool fitPlaneRHT2(const pcl::PointCloud<PointType>::Ptr &cloud,
                      std::vector<double> &normal,
                      size_t &inlier_count);

    /** \brief Validate that normal represents ground (not wall)
     * \param[in,out] normal Normal vector to check/correct
     * \return true if valid ground plane, false if wall detected
     */
    bool validateGroundNormal(std::vector<double> &normal);

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

    /** \brief Write computed ground normal vector to CSV file
     * \param[in] stamp Timestamp
     * \param[in] normal Normal vector
     * \param[in] vis_score Visibility score
     * \param[in] inlier_score Inlier score
     * \param[in] combined_score Combined score
     */
    void log_results(const ros::Time &stamp,
                     const std::vector<double> &normal,
                     double vis_score,
                     double inlier_score,
                     double combined_score);

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
