#pragma once
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>

//ROS headers
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

//other headers
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
//#include <sophus/so3.hpp>
//#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <mutex>
#include "VoxelHashMap.hpp"
#include "Threshold.hpp"
#include <ikd_Tree.h>

#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>



#include <state_estimator_msgs/Estimator.h>
#include <evaluation_msgs/Runtime.h>
#include <evaluation_msgs/Error.h>
#include <evaluation_msgs/Treesize.h>

//global configuration
    enum InitialMethod{
        KALMAN=0, IMU=1
    };
    enum MapInitMethod{
        OFF=0, COMBINE=1, SLAM=2
    };
    struct LIOConfig {
    /*--------------
        coordinate frames
            ------------*/
    std::string base_frame{"odom"};
    std::string odom_frame{"map3"};
    std::string kalman_odom_frame{"map2"};
    std::string imu_odom_frame{"map"};
    std::string imu_frame{"imu_frame"};
    std::string lidar_frame{"pandar_frame"};
    
    /*--------------
      preprocessing
      ------------*/
    // deskewing
    bool deskewing; //true: on, false: off
    // range filter
    double min_range; //lower limit for accepted point distances in m
    double max_range; //upper limit for accepted point distances in m
    //subsampling
    double vox_l_map; //voxel size for point cloud added to map in m
    double vox_l_gicp; //voxel size for point cloud utilized in GICP in m
    /*--------------
      Initial Guess Determination
                    ------------*/
    InitialMethod initial_method;
    /*--------------
    GICP Scan Matching
         ------------*/
    bool robust_kernel; //true: active, false: inactive
    int target_NN_k; // number of points found for each point in source to construct the target set
    int gicp_max_it; //maximum allowed number of iterations for the GICP loop
    double gicp_th_fitness; //convergence criterium for the fitness value
    double gicp_th_rmse; //convergence criterium for the RMSE value
    double gicp_max_corr_dist; //maximum correspondence distance for GICP scan matching in m (only used when adaptive thresholding inactive)
    double gicp_kernel_value;
    /*--------------
    Adaptive Thresholding
            ------------*/
    bool adaptive_th; //true: active, false: inactive
    double initial_th; //sigma_0 (the initial threshold) which is assumed as long as no other corrections were computed
    double min_motion_th; //delta_min (the minimum motion) that have to be measured so that a computed correction is considered in the adaptive threshold determination
    // map params
    double voxel_size;
    double overlap_map_voxel_size;
    int max_points_per_voxel;
    int overlap_map_max_points_per_voxel;
    /*--------------
    Global Map Management
            ------------*/
    MapInitMethod map_init_method; //choose map init method (no method, take and combine raw scans, apply GraphSLAM)
    double map_init_angle; //angle in degree the sphere has to be rotated so that the map initialization is performed
    double map_init_vox_l; //voxel size for point cloud utilized in initialization
    int SLAM_max_it; //maximum number of iterations allowed to be executed by GraphSLAM
    double SLAM_epsilon; //convergence threshold for GraphSLAM
    double SLAM_max_dist_2; //maximum allowed point distance in GraphSLAM
    int SLAM_min_corr; //minimum number of correspondences that need to be found so that GraphSLAM assumes two scans as overlapping
    float ikd_alpha_del; //delete criterium for the dynamic re-balancing of the ikd-Tree
    float ikd_alpha_bal; //balance criterium for the dynamic re-balancing of the ikd-Tree
    float ikd_downsample_l; //l_box (cuboid box length) of downsample boxes for the automatized point reduction
    int ikd_downsample_n; //n_box^max (maximum point number) of the downsample boxes
    bool sliding_map; //true: active, false: inactive
    double sliding_map_L; //L (length of the cuboid) of the sliding map window
    float sliding_map_gamma; //gamma (relaxation) of the sliding map window
    bool publish_clouds; //true: active, false: inactive
    bool publish_traj; //true: active, false: inactive
    bool publish_poses; //true: active, false: inactive
    bool print_runtime; //true: active, false: inactive

};

//custom PointType
namespace custom_type
{
    struct PointXYZITR //for Hesai Pandar LiDAR data
    {
        PCL_ADD_POINT4D; // quad-word XYZ
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(custom_type::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

using PointType = pcl::PointXYZ;//custom_type::PointXYZITR;//
using PointVector = KD_TREE<PointType>::PointVector;
namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}
inline double square(double x) { return x * x; }
struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};
void compare_transformation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &frame_pcl,
    std::vector<Eigen::Vector3d> &frame_eigen,
    const Sophus::SE3d &transform);

void EigenTransformation(
    std::vector<Eigen::Vector3d> &frame,
    const Sophus::SE3d &transform);

void PCLTransformation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &frame,
    const Sophus::SE3d &transform);
typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double> kd_tree_t;
class LIONode{
    public:
    //constructor
    LIONode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    private:
    //runtime analysis
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

    //configuration
    LIOConfig config_;
    //subscriber
    ros::Subscriber pc_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    //publisher
    ros::Publisher odom_pub_;
    ros::Publisher traj_all_pub_;
    ros::Publisher traj_all2_pub_;
    ros::Publisher traj_reg_pub_;
    ros::Publisher pose_all_pub_;
    ros::Publisher pose_all2_pub_;
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
    //tf stuff
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    //static transforms
    Sophus::SE3d tf_imu2lidar_;
    Sophus::SE3d tf_lidar2base_;
    //node handle
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    //callback functions
    void processPoints(const sensor_msgs::PointCloud2ConstPtr &msg);
    void processIMU(const state_estimator_msgs::EstimatorConstPtr &msg);
    void processPose(const geometry_msgs::PoseStampedConstPtr &msg);
    //publisher/tf broadcaster calls
    void publishPointClouds(bool reg_suc, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const ros::Time &stamp, Sophus::SE3d &pose);
    void publishOdometry(const ros::Time &stamp);
    void publishPosesTraj(const Sophus::SE3d &initial_guess, const Sophus::SE3d &registered_pose, bool reg_suc, const ros::Time &stamp);
    //attributes
    bool publish_clouds_;

    // _____________________
    
    // process LiDAR points
    // functions:
    bool undistort(pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, Sophus::SE3d& initial_guess);
    bool registerPoints(const pcl::PointCloud<PointType>::Ptr &pc_in, const Sophus::SE3d &initial_guess, Sophus::SE3d &registered_pose, double max_correspondence_distance, int max_num_iterations, double kernel);
    // attributes:
    Sophus::SE3d initial_guess;
    bool pose_initialized_ = false;
    Sophus::SE3d prev_init_pose_;
    Sophus::SE3d prev_pose_;
    Sophus::SE3d first_pose_;
    tf2::Vector3 lidar_offset=tf2::Vector3(-0.010, -0.001, 0.135);
    tf2::Quaternion imu2lidar_rot=tf2::Quaternion(0,0,1,0);
    //check if and how registration shall be performed
    bool check_if_registration();
    void build_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, const Sophus::SE3d &pose, const Sophus::SE3d &initial_guess, const ros::Time &stamp);
    double angle_roll_=0.0;
    double angle_pitch_=0.0;
    double angle_yaw_=0.0;
    std::deque<double> d_roll_;
    std::deque<double> d_pitch_;
    std::deque<double> d_yaw_;
    double d_roll_sum_=0.0;
    double d_yaw_sum_=0.0;
    double d_pitch_sum_=0.0;
    bool map_init_started_ = false;
    std::vector<pcl::PointCloud<PointType>> map_pc_buffer_;
    std::vector<std::vector<Eigen::Vector3d>> map_pc_eigen_buffer_;
    std::vector<Sophus::SE3d> map_pose_buffer_;
    bool map_initialized_ = false;
    int counter_ = 0;
    int init_counter_ = 0;
    int num_iterations_between_ = 1;
    int no_match_counter_ = 0;
    double rel_cov_th_=0.9;
    double max_distance_ = 3.0;
    std::mutex mtx_kdtree_;
    std::mutex mtx_pose_;
    std::vector<BoxPointType> cub_needrm_;
    Eigen::Vector3d pos_lid_ = Eigen::Vector3d::Zero();
    BoxPointType local_map_points_;
    bool localmap_initialized_ = false;
    //maps and poses
    KD_TREE<PointType>::Ptr ikdtree_ptr_;
    VoxelHashMap overlap_map_=VoxelHashMap(config_.overlap_map_voxel_size, config_.max_range, config_.overlap_map_max_points_per_voxel);
    std::vector<Sophus::SE3d> poses_;
    //adaptive threshold
    AdaptiveThreshold adaptive_threshold_ = AdaptiveThreshold(config_.initial_th, config_.min_motion_th, config_.max_range);
    
    
    //GICP
    double epsilon_ = 1e-3;
    // ---- GICP-specific helpers declared here and implemented in registration_gicp.cpp ----
    // Apply SE3 to points/covariances in-place
    void transformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points);
    void transformCovariances(const Sophus::SE3d &T, std::vector<Eigen::Matrix3d> &covariances);
    // Rotation that aligns e1 to x (used for per-point covariance orientation)
    Eigen::Matrix3d GetRotationFromE1ToX(const Eigen::Vector3d &x);
    // Main GICP registration routine (variant used in registration_gicp.cpp)
    bool registerPointsGICP(const pcl::PointCloud<PointType>::Ptr &pc_in,
                            const Sophus::SE3d &initial_guess,
                            Sophus::SE3d &registered_pose,
                            double kernel, double max_correspondence_distance,
                            int max_num_iterations,
                            double sigma,
                            double &matching_error);
    // Build linear system for GICP
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> BuildLinearSystemGICP(
        const std::vector<Eigen::Vector3d> &source,
        const std::vector<Eigen::Vector3d> &target,
        const std::vector<Eigen::Matrix3d> &c_source,
        const std::vector<Eigen::Matrix3d> &c_target,
        const std::vector<Eigen::Vector2i> &corres,
        double kernel);
    // Parallel accumulation of JTJ, JTr
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> ComputeJTJandJTr(
        std::function<
            void(int,
                 std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &,
                 std::vector<double> &,
                 std::vector<double> &)> f,
        int iteration_num,
        bool verbose = true);
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> ComputeJTJandJTr2(
        std::function<
                void(int,
                     std::vector<Eigen::Vector6d> &,
                     std::vector<double> &,
                     std::vector<double> &)> f,
        int iteration_num,
        bool verbose =true);
    // Per-point covariance (KD-tree constructed internally)
    std::vector<Eigen::Matrix3d> computePerPointCovariances(const std::vector<Eigen::Vector3d> &points);
    std::vector<Eigen::Matrix3d> computePerPointCovarianceskdTree(const std::vector<Eigen::Vector3d> &points, const kd_tree_t &kdtree);

    // Low-level covariance from neighbor indices
    Eigen::Matrix3d computeCovariance(const std::vector<Eigen::Vector3d> &points,
                                      const size_t *indices,
                                      size_t count);
    // Normal estimation (KD-tree internal)
    std::vector<Eigen::Vector3d> computeNormals(const std::vector<Eigen::Vector3d> &points);
    std::vector<Eigen::Vector3d> computeNormalskdTree(const std::vector<Eigen::Vector3d> &points, const kd_tree_t &kdtree);
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> construct_source_and_target_pc(const pcl::PointCloud<PointType>::Ptr &pc_in);
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> construct_source_and_target_pc_box(const pcl::PointCloud<PointType>::Ptr &pc_in);
    void find_correspondences_and_calculate_error(std::vector<Eigen::Vector3d> &source, std::vector<Eigen::Vector2i> &correspondences, const kd_tree_t &target_kdtree, double &max_correspondence_distance, double &error2);
    double ComputeRMSE(
        const std::vector<Eigen::Vector3d> &source,
        const std::vector<Eigen::Vector3d> &target,
        const std::vector<Eigen::Matrix3d> &covariances_source,
        const std::vector<Eigen::Matrix3d> &covariances_target,
        const std::vector<Eigen::Vector2i> &corres) const;
    //nanoGICP
    bool registerPointsNanoGICP(const pcl::PointCloud<PointType>::Ptr &pc_in, const Sophus::SE3d &initial_guess, Sophus::SE3d &registered_pose, double max_correspondence_distance, int max_num_iterations, double sigma);
    

    //process IMU measurements
    struct ImuMeas {
        double stamp;
        tf2::Vector3 ang_vel;
        tf2::Quaternion orientation;
        Sophus::SE3d pose;
    };
    ImuMeas imu_meas_;
    boost::circular_buffer<ImuMeas> imu_buffer_;
    std::mutex mtx_imu_;
    std::condition_variable cv_imu_stamp_;
    //store poses and paths
    nav_msgs::Path all_path_msg_; //all poses (initial if not registered)
    nav_msgs::Path all_path2_msg_; //all poses (up to now if not registered)
    nav_msgs::Path reg_path_msg_; //only successful registration
    //DEBUGGING
    double imu_x_ = 0;
    double imu_y_ = 0;
    double imu_z_ = 0;
    int imu_num_ = 0;
    void preprocess(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out, double start_time);
    //helpers
    Sophus::SE3d LookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time = ros::Time(0)) const;
    void voxelize(const pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out, float voxel_size);
    void voxelize_XYZITR(const pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_out, float voxel_size);
    void dist_filter(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_out, double max_range, double min_range);
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d> BuildLinearSystem(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double kernel);
    
    void move_map(int &kdtree_delete_counter, const Sophus::SE3d &current_pose);
    std::tuple<Sophus::SE3d, Sophus::SE3d>  getInitialGuess(const ros::Time &timestamp);
    double GetAdaptiveThreshold();
    bool HasMoved();
    bool no_match = false;
    
    //conversions
    inline Sophus::SE3d transformToSophus(const geometry_msgs::TransformStamped &transform) const{
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
    }
    inline Sophus::SE3d::QuaternionType eulerToQuat(double roll, double pitch, double yaw){
        //only valid for small angles
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
    inline geometry_msgs::Transform sophusToTransform(const Sophus::SE3d &T) {
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
    inline geometry_msgs::Pose sophusToPose(const Sophus::SE3d &T) {
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