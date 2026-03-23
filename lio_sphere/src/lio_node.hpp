#pragma once
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>

// ROS headers
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h> //only for debugging
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64MultiArray.h>

// other headers
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>

#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <mutex>
#include "Threshold.hpp"
#include <ikd_Tree.h>

#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>

#include <state_estimator_msgs/Estimator.h>
#include <evaluation_msgs/Runtime.h>
#include <evaluation_msgs/Error.h>
#include <evaluation_msgs/Treesize.h>

// global configuration
enum InitialMethod
{
    KALMAN = 0,
    IMU = 1
};
enum MapInitMethod
{
    OFF = 0,
    COMBINE = 1,
    SLAM = 2
};
struct LIOConfig
{
    /*--------------
        coordinate frames
            ------------*/
    std::string base_frame{"center"};
    std::string odom_frame{"map_lio"};
    std::string kalman_odom_frame{"map_lkf"};
    std::string imu_odom_frame{"map_imu"};
    std::string imu_frame{"imu_frame"};
    std::string lidar_frame{"pandar_frame"};

    /*--------------
      preprocessing
      ------------*/
    // deskewing
    bool deskewing; // true: on, false: off
    // range filter
    double min_range; // lower limit for accepted point distances in m
    double max_range; // upper limit for accepted point distances in m
    // subsampling
    double vox_l_map;  // voxel size for point cloud added to map in m
    double vox_l_gicp; // voxel size for point cloud utilized in GICP in m
    /*--------------
      Initial Guess Determination
                    ------------*/
    InitialMethod initial_method;
    /*--------------
    GICP Scan Matching
         ------------*/
    bool robust_kernel;        // true: active, false: inactive
    int target_NN_k;           // number of points found for each point in source to construct the target set
    int gicp_max_it;           // maximum allowed number of iterations for the GICP loop
    double gicp_th_fitness;    // convergence criterium for the fitness value
    double gicp_th_rmse;       // convergence criterium for the RMSE value
    double gicp_max_corr_dist; // maximum correspondence distance for GICP scan matching in m (only used when adaptive thresholding inactive)
    double gicp_kernel_value;
    /*--------------
    Adaptive Thresholding
            ------------*/
    bool adaptive_th;     // true: active, false: inactive
    double initial_th;    // sigma_0 (the initial threshold) which is assumed as long as no other corrections were computed
    double min_motion_th; // delta_min (the minimum motion) that have to be measured so that a computed correction is considered in the adaptive threshold determination
    /*--------------
    Global Map Management
            ------------*/
    MapInitMethod map_init_method; // choose map init method (no method, take and combine raw scans, apply GraphSLAM)
    double map_init_angle;         // angle in degree the sphere has to be rotated so that the map initialization is performed
    double map_init_vox_l;         // voxel size for point cloud utilized in initialization
    int SLAM_max_it;               // maximum number of iterations allowed to be executed by GraphSLAM
    double SLAM_epsilon;           // convergence threshold for GraphSLAM
    double SLAM_max_dist_2;        // maximum allowed point distance in GraphSLAM
    int SLAM_min_corr;             // minimum number of correspondences that need to be found so that GraphSLAM assumes two scans as overlapping
    float ikd_alpha_del;           // delete criterium for the dynamic re-balancing of the ikd-Tree
    float ikd_alpha_bal;           // balance criterium for the dynamic re-balancing of the ikd-Tree
    float ikd_downsample_l;        // l_box (cuboid box length) of downsample boxes for the automatized point reduction
    int ikd_downsample_n;          // n_box^max (maximum point number) of the downsample boxes
    bool sliding_map;              // true: active, false: inactive
    double sliding_map_L;          // L (length of the cuboid) of the sliding map window
    float sliding_map_gamma;       // gamma (relaxation) of the sliding map window
    bool publish_clouds;           // true: active, false: inactive
    bool publish_traj;             // true: active, false: inactive
    bool publish_poses;            // true: active, false: inactive
    bool print_runtime;            // true: active, false: inactive
};

// custom PointType
namespace custom_type
{
    struct PointXYZITR // for Hesai Pandar LiDAR data
    {
        PCL_ADD_POINT4D; // quad-word XYZ
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(custom_type::PointXYZITR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

using PointType = pcl::PointXYZ; // custom_type::PointXYZITR;//
using PointVector = KD_TREE<PointType>::PointVector;
namespace Eigen
{
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}
/** @brief Returns the square of a value.
 * @param x The value to square.
 * @return x * x.
 */
inline double square(double x) { return x * x; }
typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double> kd_tree_t;
class LIONode
{
public:
    /** @brief Constructs the LIO node, loads all parameters from the parameter server,
     *         sets up subscribers/publishers, and initializes internal data structures.
     * @param nh Public ROS node handle for topic advertisement and subscription.
     * @param pnh Private ROS node handle for parameter retrieval.
     */
    LIONode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    // runtime analysis
    std::chrono::high_resolution_clock::time_point runtime_t0;
    std::chrono::high_resolution_clock::time_point runtime_t1;
    std::chrono::high_resolution_clock::time_point runtime_pp_t0;
    std::chrono::high_resolution_clock::time_point runtime_pp_t1;
    std::chrono::high_resolution_clock::time_point runtime_ig_t0;
    std::chrono::high_resolution_clock::time_point runtime_ig_t1;
    std::chrono::high_resolution_clock::time_point runtime_gicp_t0;
    std::chrono::high_resolution_clock::time_point runtime_gicp_t1;
    std::chrono::high_resolution_clock::time_point runtime_map_t0;
    std::chrono::high_resolution_clock::time_point runtime_map_t1;

    // configuration
    LIOConfig config_;
    // subscriber
    ros::Subscriber pc_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    // publisher
    ros::Publisher traj_all_pub_;
    ros::Publisher traj_all2_pub_;
    ros::Publisher traj_reg_pub_;
    ros::Publisher pose_all_pub_;
    ros::Publisher pose_reg_pub_;
    ros::Publisher init_pose_pub_;

    ros::Publisher pc_pub_;
    ros::Publisher pc_reg_pub_;
    ros::Publisher pc_debug_pub_;
    ros::Publisher map_pub_;
    // GICP debug publishers
    ros::Publisher source_pc_pub_;
    ros::Publisher target_pc_pub_;

    ros::Publisher runtime_pub_;
    ros::Publisher error_pub_;
    ros::Publisher treesize_pub_;
    // tf stuff
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    // static transforms
    Sophus::SE3d tf_imu2lidar_;
    Sophus::SE3d tf_lidar2base_;
    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // callback functions

    /** @brief Main LiDAR callback. Preprocesses the point cloud, computes an initial guess,
     *         performs GICP registration, updates the map, and publishes results.
     * @param msg Incoming ROS PointCloud2 message.
     */
    void processPoints(const sensor_msgs::PointCloud2ConstPtr &msg);

    /** @brief IMU callback. Transforms angular velocity into the LiDAR frame, extracts pose,
     *         and pushes the measurement into the circular IMU buffer.
     * @param msg Incoming estimator message containing IMU data and pose.
     */
    void processIMU(const state_estimator_msgs::EstimatorConstPtr &msg);

    /** @brief Pose callback (currently unused placeholder).
     * @param msg Incoming PoseStamped message.
     */
    void processPose(const geometry_msgs::PoseStampedConstPtr &msg);
    // publisher/tf broadcaster calls

    /** @brief Publishes the registered and debug point clouds as ROS messages.
     * @param reg_suc Whether the registration was successful.
     * @param pc The point cloud to publish.
     * @param stamp Timestamp for the published messages.
     * @param pose The current estimated pose used to transform the cloud into the map frame.
     */
    void publishPointClouds(bool reg_suc, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const ros::Time &stamp, Sophus::SE3d &pose);

    /** @brief Publishes the current odometry as a TF transform.
     * @param stamp Timestamp for the broadcasted transform.
     */
    void publishOdometry(const ros::Time &stamp);

    /** @brief Publishes trajectory and pose path messages for visualisation in RViz.
     * @param initial_guess The initial guess pose before registration.
     * @param registered_pose The pose after GICP registration.
     * @param reg_suc Whether the registration was successful.
     * @param stamp Timestamp for the published messages.
     */
    void publishPosesTraj(const Sophus::SE3d &initial_guess, const Sophus::SE3d &registered_pose, bool reg_suc, const ros::Time &stamp);
    // _____________________

    /** @brief Removes motion distortion from a LiDAR sweep using interpolated IMU angular velocities.
     *         Each point is rotated back to the sweep start frame based on its individual timestamp.
     * @param pc_in Input point cloud with per-point timestamps and ring indices.
     * @param pc_out Output undistorted point cloud (PointXYZ).
     * @param initial_guess [out] Set to the IMU pose at the sweep start.
     * @return True on success, false if the IMU buffer does not cover the sweep.
     */
    bool undistort(pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, Sophus::SE3d &initial_guess);
    // attributes:
    Sophus::SE3d initial_guess;
    bool pose_initialized_ = false;
    Sophus::SE3d prev_init_pose_;
    Sophus::SE3d prev_pose_;
    tf2::Quaternion imu2lidar_rot = tf2::Quaternion(0, 0, 1, 0);
    /** @brief Accumulates scans during the map initialisation phase and triggers map building
     *         (either raw combination or GraphSLAM) once the configured rotation angle is reached.
     * @param pc_in Voxelized point cloud to add to the initialisation buffer.
     * @param pose The current registered pose in the map frame.
     * @param initial_guess The relative pose difference used to track accumulated rotation.
     * @param stamp Timestamp of the current scan.
     */
    void build_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, const Sophus::SE3d &pose, const Sophus::SE3d &initial_guess, const ros::Time &stamp);
    double angle_roll_ = 0.0;
    double angle_pitch_ = 0.0;
    std::deque<double> d_roll_;
    std::deque<double> d_pitch_;
    double d_roll_sum_ = 0.0;
    double d_pitch_sum_ = 0.0;
    bool map_init_started_ = false;
    std::vector<pcl::PointCloud<PointType>> map_pc_buffer_;
    std::vector<std::vector<Eigen::Vector3d>> map_pc_eigen_buffer_;
    std::vector<Sophus::SE3d> map_pose_buffer_;
    bool map_initialized_ = false;
    std::mutex mtx_kdtree_;
    std::mutex mtx_pose_;
    std::vector<BoxPointType> cub_needrm_;
    Eigen::Vector3d pos_lid_ = Eigen::Vector3d::Zero();
    BoxPointType local_map_points_;
    bool localmap_initialized_ = false;
    // maps and poses
    KD_TREE<PointType>::Ptr ikdtree_ptr_;
    std::vector<Sophus::SE3d> poses_;
    // adaptive threshold
    AdaptiveThreshold adaptive_threshold_ = AdaptiveThreshold(config_.initial_th, config_.min_motion_th, config_.max_range);

    // GICP
    double epsilon_ = 1e-3;
    // ---- GICP-specific helpers declared here and implemented in registration_gicp.cpp ----

    /** @brief Applies an SE3 transformation to each 3D point in-place using TBB parallelism.
     * @param T The SE3 transformation to apply.
     * @param points [in/out] Vector of 3D points to transform.
     */
    void transformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points);

    /** @brief Rotates each 3x3 covariance matrix by the rotation part of T in-place using TBB.
     * @param T The SE3 transformation whose rotation is applied as R * C * R^T.
     * @param covariances [in/out] Vector of covariance matrices to transform.
     */
    void transformCovariances(const Sophus::SE3d &T, std::vector<Eigen::Matrix3d> &covariances);

    /** @brief Computes a rotation matrix that maps the unit vector e1=(1,0,0) onto x.
     *         Used to orient per-point covariance ellipsoids along the estimated normal.
     * @param x Target unit direction vector.
     * @return 3x3 rotation matrix aligning e1 to x.
     */
    Eigen::Matrix3d GetRotationFromE1ToX(const Eigen::Vector3d &x);

    /** @brief Performs Generalized ICP registration of a source point cloud against the global map.
     *         Constructs source/target sets from the ikd-tree, estimates normals and covariances,
     *         and iteratively minimises the GICP cost function.
     * @param pc_in Input source point cloud (already voxelized).
     * @param initial_guess Initial SE3 pose estimate for the source cloud.
     * @param registered_pose [out] The refined SE3 pose after registration.
     * @param kernel Robust kernel parameter for outlier weighting.
     * @param max_correspondence_distance Maximum allowed distance between corresponding points.
     * @param max_num_iterations Maximum number of ICP iterations.
     * @param matching_error [out] The final inlier RMSE after convergence.
     * @return True if registration succeeded, false otherwise.
     */
    bool registerPointsGICP(const pcl::PointCloud<PointType>::Ptr &pc_in,
                            const Sophus::SE3d &initial_guess,
                            Sophus::SE3d &registered_pose,
                            double kernel, double max_correspondence_distance,
                            int max_num_iterations,
                            double &matching_error);

    /** @brief Builds the 6x6 normal equation system (JTJ, JTr) for a single GICP iteration.
     *         Computes weighted Jacobians and residuals from point-to-plane distances under
     *         the combined source/target covariance model.
     * @param source Source point positions (in current ICP frame).
     * @param target Target point positions.
     * @param c_source Source covariance matrices.
     * @param c_target Target covariance matrices.
     * @param corres Correspondence pairs as (source_index, target_index).
     * @param kernel Robust kernel parameter.
     * @return Tuple of (JTJ, JTr, sum_of_squared_residuals).
     */
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> BuildLinearSystemGICP(
        const std::vector<Eigen::Vector3d> &source,
        const std::vector<Eigen::Vector3d> &target,
        const std::vector<Eigen::Matrix3d> &c_source,
        const std::vector<Eigen::Matrix3d> &c_target,
        const std::vector<Eigen::Vector2i> &corres,
        double kernel);

    /** @brief Accumulates JTJ and JTr in parallel using TBB by evaluating a per-element
     *         Jacobian/residual functor over all correspondences.
     * @param f Functor called as f(index, J_rows, residuals, weights) for each correspondence.
     * @param iteration_num Total number of correspondences to process.
     * @return Tuple of (JTJ, JTr, sum_of_squared_residuals).
     */
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> ComputeJTJandJTr(
        std::function<
            void(int,
                 std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &,
                 std::vector<double> &,
                 std::vector<double> &)>
            f,
        int iteration_num);

    /** @brief Estimates per-point 3x3 covariance matrices from k-nearest neighbours.
     *         Internally builds a kd-tree over the input points.
     * @param points Input 3D point set.
     * @return Vector of 3x3 covariance matrices, one per input point.
     */
    std::vector<Eigen::Matrix3d> computePerPointCovariances(const std::vector<Eigen::Vector3d> &points);

    /** @brief Estimates per-point 3x3 covariance matrices using a pre-built kd-tree.
     * @param points Input 3D point set.
     * @param kdtree Pre-built kd-tree over the same point set.
     * @return Vector of 3x3 covariance matrices, one per input point.
     */
    std::vector<Eigen::Matrix3d> computePerPointCovarianceskdTree(const std::vector<Eigen::Vector3d> &points, const kd_tree_t &kdtree);

    /** @brief Computes the sample covariance matrix from a subset of points given by indices.
     *         Uses a cumulant-based formulation for numerical efficiency.
     * @param points Full point set.
     * @param indices Pointer to an array of indices into points.
     * @param count Number of indices.
     * @return 3x3 covariance matrix (identity if count < 3).
     */
    Eigen::Matrix3d computeCovariance(const std::vector<Eigen::Vector3d> &points,
                                      const size_t *indices,
                                      size_t count);

    /** @brief Estimates surface normals for each point by computing the eigenvector of smallest
     *         eigenvalue of the per-point covariance. Builds a kd-tree internally.
     * @param points Input 3D point set.
     * @return Vector of unit normal vectors, one per input point.
     */
    std::vector<Eigen::Vector3d> computeNormals(const std::vector<Eigen::Vector3d> &points);

    /** @brief Estimates surface normals using a pre-built kd-tree, orienting them toward the sensor origin.
     * @param points Input 3D point set.
     * @param kdtree Pre-built kd-tree over the same point set.
     * @return Vector of unit normal vectors, one per input point.
     */
    std::vector<Eigen::Vector3d> computeNormalskdTree(const std::vector<Eigen::Vector3d> &points, const kd_tree_t &kdtree);

    /** @brief Constructs source and target point sets for GICP by querying the ikd-tree
     *         for nearest neighbours of each input point.
     * @param pc_in Input point cloud (already transformed by the initial guess).
     * @return Pair of (source_points, target_points) as Eigen vectors.
     */
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> construct_source_and_target_pc(const pcl::PointCloud<PointType>::Ptr &pc_in);

    /** @brief Finds nearest-neighbour correspondences between source points and the target kd-tree,
     *         filtering by maximum distance, and accumulates the squared error.
     * @param source [in] Source point positions.
     * @param correspondences [out] Resulting correspondence pairs (source_idx, target_idx).
     * @param target_kdtree Pre-built kd-tree over the target points.
     * @param max_correspondence_distance Maximum allowed correspondence distance.
     * @param error2 [in/out] Accumulated sum of squared correspondence distances.
     */
    void find_correspondences_and_calculate_error(std::vector<Eigen::Vector3d> &source, std::vector<Eigen::Vector2i> &correspondences, const kd_tree_t &target_kdtree, double &max_correspondence_distance, double &error2);

    /** @brief Computes the GICP-weighted RMSE over all correspondences using the combined
     *         source and target covariance matrices.
     * @param source Source point positions.
     * @param target Target point positions.
     * @param covariances_source Source covariance matrices.
     * @param covariances_target Target covariance matrices.
     * @param corres Correspondence index pairs.
     * @return Root mean squared error (0.0 if no correspondences).
     */
    double ComputeRMSE(
        const std::vector<Eigen::Vector3d> &source,
        const std::vector<Eigen::Vector3d> &target,
        const std::vector<Eigen::Matrix3d> &covariances_source,
        const std::vector<Eigen::Matrix3d> &covariances_target,
        const std::vector<Eigen::Vector2i> &corres) const;

    // process IMU measurements
    struct ImuMeas
    {
        double stamp;
        tf2::Vector3 ang_vel;
        tf2::Quaternion orientation;
        Sophus::SE3d pose;
    };
    ImuMeas imu_meas_;
    boost::circular_buffer<ImuMeas> imu_buffer_;
    std::mutex mtx_imu_;
    std::condition_variable cv_imu_stamp_;
    // store poses and paths
    nav_msgs::Path all_path_msg_;  // all poses (initial if not registered)
    nav_msgs::Path all_path2_msg_; // all poses (up to now if not registered)
    nav_msgs::Path reg_path_msg_;  // only successful registration
    // helpers

    /** @brief Looks up a TF2 transform between two frames and converts it to Sophus::SE3d.
     * @param target_frame Target TF frame name.
     * @param source_frame Source TF frame name.
     * @param time Timestamp for the transform lookup (default: latest available).
     * @return The SE3 transform from source to target (identity on failure).
     */
    Sophus::SE3d LookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time = ros::Time(0)) const;

    /** @brief Downsamples a point cloud using a VoxelGrid filter with uniform cell size.
     * @param pc_in Input point cloud.
     * @param pc_out [out] Output voxelized point cloud.
     * @param voxel_size Side length of each voxel cube in metres.
     */
    void voxelize(const pcl::PointCloud<PointType>::Ptr &pc_in, pcl::PointCloud<PointType>::Ptr &pc_out, float voxel_size);

    /** @brief Filters points by Euclidean distance from the origin (custom point type overload).
     * @param pc_in Input point cloud.
     * @param pc_out [out] Output cloud containing only points within the distance range.
     * @param max_range Maximum allowed point distance in metres.
     * @param min_range Minimum allowed point distance in metres.
     */
    void dist_filter(pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_in, pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_out, double max_range, double min_range);

    /** @brief Filters points by Euclidean distance from the origin (PointXYZ overload).
     * @param pc_in Input point cloud.
     * @param pc_out [out] Output cloud containing only points within the distance range.
     * @param max_range Maximum allowed point distance in metres.
     * @param min_range Minimum allowed point distance in metres.
     */
    void dist_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, double max_range, double min_range);

    /** @brief Implements the sliding-map strategy by deleting points from the ikd-tree
     *         that fall outside a moving cuboid window centred on the current pose.
     * @param kdtree_delete_counter [out] Number of points deleted from the tree.
     * @param current_pose Current estimated pose used to determine the window centre.
     */
    void move_map(int &kdtree_delete_counter, const Sophus::SE3d &current_pose);

    /** @brief Computes the initial guess for GICP by looking up the external odometry (Kalman/IMU)
     *         and composing it with the previous registered pose.
     * @param timestamp Timestamp at which to query the odometry transform.
     * @return Tuple of (predicted_pose_in_map, relative_pose_difference).
     */
    std::tuple<Sophus::SE3d, Sophus::SE3d> getInitialGuess(const ros::Time &timestamp);

    // conversions

    /** @brief Converts a ROS TransformStamped message to a Sophus SE3d object.
     * @param transform The ROS transform message.
     * @return Equivalent Sophus::SE3d.
     */
    inline Sophus::SE3d transformToSophus(const geometry_msgs::TransformStamped &transform) const
    {
        const auto &t = transform.transform;
        return Sophus::SE3d(
            Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
            Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
    }
    /** @brief Converts Euler angles (roll, pitch, yaw) to a unit quaternion.
     * @param roll Rotation around the X axis in radians.
     * @param pitch Rotation around the Y axis in radians.
     * @param yaw Rotation around the Z axis in radians.
     * @return The corresponding unit quaternion.
     */
    inline Sophus::SE3d::QuaternionType eulerToQuat(double roll, double pitch, double yaw)
    {
        // only valid for small angles
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        double q_w = cr * cp * cy + sr * sp * sy;
        double q_x = sr * cp * cy - cr * sp * sy;
        double q_y = cr * sp * cy + sr * cp * sy;
        double q_z = cr * cp * sy - sr * sp * cy;
        return Sophus::SE3d::QuaternionType(q_w, q_x, q_y, q_z);
    }
    /** @brief Converts a Sophus SE3d object to a ROS geometry_msgs::Transform.
     * @param T The Sophus SE3d pose.
     * @return Equivalent ROS Transform message.
     */
    inline geometry_msgs::Transform sophusToTransform(const Sophus::SE3d &T)
    {
        geometry_msgs::Transform t;
        t.translation.x = T.translation().x();
        t.translation.y = T.translation().y();
        t.translation.z = T.translation().z();

        Eigen::Quaterniond q(T.so3().unit_quaternion());
        t.rotation.x = q.x();
        t.rotation.y = q.y();
        t.rotation.z = q.z();
        t.rotation.w = q.w();

        return t;
    }
    /** @brief Converts a Sophus SE3d object to a ROS geometry_msgs::Pose.
     * @param T The Sophus SE3d pose.
     * @return Equivalent ROS Pose message.
     */
    inline geometry_msgs::Pose sophusToPose(const Sophus::SE3d &T)
    {
        geometry_msgs::Pose t;
        t.position.x = T.translation().x();
        t.position.y = T.translation().y();
        t.position.z = T.translation().z();

        Eigen::Quaterniond q(T.so3().unit_quaternion());
        t.orientation.x = q.x();
        t.orientation.y = q.y();
        t.orientation.z = q.z();
        t.orientation.w = q.w();

        return t;
    }
};