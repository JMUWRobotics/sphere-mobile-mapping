#pragma once
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

//other headers
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
//#include <sophus/so3.hpp>
//#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include "VoxelHashMap.hpp"
#include "Threshold.hpp"
#include <ikd_Tree.h>

#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>



#include <state_estimator_msgs/Estimator.h>

//global configuration
struct LIOConfig {
    // map params
    double voxel_size = 0.5;
    double overlap_map_voxel_size = 3;
    double max_range = 30.0;
    double min_range = 0.2;
    int max_points_per_voxel = 1;
    int overlap_map_max_points_per_voxel = 1;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // Motion compensation
    bool deskew = false;
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
    LIONode(const ros::NodeHandle &nh);
    private:
    //configuration
    LIOConfig config_;
    //subscriber
    ros::Subscriber pc_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    //publisher
    ros::Publisher odom_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher pc_pub_;
    ros::Publisher pc_debug_pub_;
    ros::Publisher map_pub_;
    // GICP debug publishers
    ros::Publisher source_pc_pub_;
    ros::Publisher target_pc_pub_;
    //tf stuff
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    //static transforms
    Sophus::SE3d tf_imu2lidar_;
    Sophus::SE3d tf_lidar2base_;
    //node handle
    ros::NodeHandle nh_;
    //callback functions
    void processPoints(const sensor_msgs::PointCloud2ConstPtr &msg);
    void processIMU(const state_estimator_msgs::EstimatorConstPtr &msg);
    void processPose(const geometry_msgs::PoseStampedConstPtr &msg);
    //publisher/tf broadcaster calls
    void publishPointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const ros::Time &stamp, Sophus::SE3d &pose);
    void publishOdometry(const ros::Time &stamp);
    //attributes
    std::string base_frame_{"odom"};
    std::string odom_frame_{"map3"};
    std::string prev_odom_frame_{"map2"};
    std::string imu_frame_{"imu_frame"};
    std::string lidar_frame_{"pandar_frame"};
    bool publish_clouds_;

    // _____________________
    
    // process LiDAR points
    // functions:
    bool distort(pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, Sophus::SE3d& initial_guess);
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
    //maps and poses
    KD_TREE<PointType>::Ptr ikdtree_ptr_;
    VoxelHashMap overlap_map_=VoxelHashMap(config_.overlap_map_voxel_size, config_.max_range, config_.overlap_map_max_points_per_voxel);
    std::vector<Sophus::SE3d> poses_;
    //adaptive threshold
    AdaptiveThreshold adaptive_threshold_ = AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
    
    
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
                            bool &max_it_reached, double max_correspondence_distance,
                            int max_num_iterations,
                            double sigma);
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

    //DEBUGGING
    double imu_x_ = 0;
    double imu_y_ = 0;
    double imu_z_ = 0;
    int imu_num_ = 0;
    void preprocess(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out, double start_time);
    //helpers
    Sophus::SE3d LookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time = ros::Time(0)) const;
    void voxelize(const pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out, float voxel_size);
    void dist_filter(pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out, double max_range, double min_range);
    std::tuple<Eigen::Matrix6d, Eigen::Vector6d> BuildLinearSystem(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double kernel);
    
    void move_map(int &kdtree_delete_counter, const Sophus::SE3d &current_pose);
    std::tuple<Sophus::SE3d, Sophus::SE3d>  getInitialGuess(const ros::Time &timestamp);
    double GetAdaptiveThreshold();
    bool HasMoved();
    
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
};