#include "lio_node.hpp"
#include "graph_slam.hpp"
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

/*-------------------
|   INITIALIZATION  |
-------------------*/

LIONode::LIONode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), tf2_listener_(tf2_buffer_)
{
    // get parameters
    pnh_.param("deskewing", config_.deskewing, true);
    pnh_.param("min_range", config_.min_range, 0.5);
    pnh_.param("max_range", config_.max_range, 30.0);
    pnh_.param("vox_l_map", config_.vox_l_map, 0.35);
    pnh_.param("vox_l_gicp", config_.vox_l_gicp, 0.75);

    if (pnh_.hasParam("initial_method"))
    {
        std::string initial_method;
        pnh_.getParam("initial_method", initial_method);
        if (initial_method == "KALMAN" || initial_method == "kalman")
            config_.initial_method = InitialMethod::KALMAN;
        if (initial_method == "IMU" || initial_method == "imu")
            config_.initial_method = InitialMethod::IMU;
    }

    pnh_.param("robust_kernel", config_.robust_kernel, true);
    pnh_.param("target_NN_k", config_.target_NN_k, 5);
    pnh_.param("gicp_th_fitness", config_.gicp_th_fitness, 1e-6);
    pnh_.param("gicp_th_rmse", config_.gicp_th_rmse, 1e-6);
    pnh_.param("gicp_max_corr_dist", config_.gicp_max_corr_dist, 3.0);
    pnh_.param("gicp_max_it", config_.gicp_max_it, 20);
    pnh_.param("gicp_kernel_value", config_.gicp_kernel_value, 3.0);

    pnh_.param("adaptive_th", config_.adaptive_th, true);
    pnh_.param("initial_th", config_.initial_th, 2.0);
    pnh_.param("min_motion_th", config_.min_motion_th, 0.1);

    if (pnh_.hasParam("map_init_method"))
    {
        std::string map_init_method;
        pnh_.getParam("map_init_method", map_init_method);
        if (map_init_method == "OFF" || map_init_method == "off")
            config_.map_init_method = MapInitMethod::OFF;
        if (map_init_method == "COMBINE" || map_init_method == "combine")
            config_.map_init_method = MapInitMethod::COMBINE;
        if (map_init_method == "SLAM" || map_init_method == "slam")
            config_.map_init_method = MapInitMethod::SLAM;
    }

    pnh_.param("map_init_angle", config_.map_init_angle, 360.0);
    pnh_.param("map_init_vox_l", config_.map_init_vox_l, 0.9);

    pnh_.param("SLAM_max_it", config_.SLAM_max_it, 10);
    pnh_.param("SLAM_epsilon", config_.SLAM_epsilon, 0.01);
    pnh_.param("SLAM_max_dist2", config_.SLAM_max_dist_2, 0.4);
    pnh_.param("SLAM_min_corr", config_.SLAM_min_corr, 100);

    pnh_.param("ikd_alpha_bal", config_.ikd_alpha_bal, 0.6f);
    pnh_.param("ikd_alpha_del", config_.ikd_alpha_del, 0.5f);
    pnh_.param("ikd_downsample_l", config_.ikd_downsample_l, 0.5f);
    pnh_.param("ikd_downsample_n", config_.ikd_downsample_n, 5);

    pnh_.param("sliding_map", config_.sliding_map, true);
    pnh_.param("sliding_map_L", config_.sliding_map_L, 150.0);
    pnh_.param("sliding_map_gamma", config_.sliding_map_gamma, 1.2f);

    pnh_.param("publish_clouds", config_.publish_clouds, true);
    pnh_.param("publish_traj", config_.publish_traj, true);
    pnh_.param("publish_poses", config_.publish_poses, true);
    pnh_.param("print_runtime", config_.print_runtime, true);

    // mutual constraints
    if (!config_.adaptive_th)
        config_.robust_kernel = false;
    // subscriptions
    pc_sub_ = nh_.subscribe("/points_in", 10, &LIONode::processPoints, this, ros::TransportHints().tcpNoDelay());
    imu_sub_ = nh_.subscribe("/imu_in", 10, &LIONode::processIMU, this, ros::TransportHints().tcpNoDelay());
    pose_sub_ = nh_.subscribe("/lkf/pose", 10, &LIONode::processPose, this, ros::TransportHints().tcpNoDelay());
    // publishers
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_out", 5);
    pc_reg_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/only_reg_points_out", 5);
    pc_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_debug_out", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_out", 1);
    traj_all_pub_ = nh_.advertise<nav_msgs::Path>("/full_traj_out", 1);
    traj_all2_pub_ = nh_.advertise<nav_msgs::Path>("/full2_traj_out", 1);
    traj_reg_pub_ = nh_.advertise<nav_msgs::Path>("/only_reg_traj_out", 1);
    pose_all_pub_ = nh_.advertise<nav_msgs::Path>("/all_pose_out", 5);
    // pose_all2_pub_ = nh_.advertise<nav_msgs::Path>("/full2_pose_out", 1);
    pose_reg_pub_ = nh_.advertise<nav_msgs::Path>("/only_reg_pose_out", 5);
    init_pose_pub_ = nh_.advertise<nav_msgs::Path>("/initial_guess", 5);
    // GICP debug clouds
    source_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gicp_source", 1);
    target_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gicp_target", 1);

    runtime_pub_ = nh_.advertise<evaluation_msgs::Runtime>("/runtimes", 5);
    error_pub_ = nh_.advertise<evaluation_msgs::Error>("/gicp_error", 5);
    treesize_pub_ = nh_.advertise<evaluation_msgs::Treesize>("/treesize", 5);
    // store static transforms
    tf_imu2lidar_ = LookupTransform(config_.lidar_frame, config_.imu_frame);
    tf_lidar2base_ = LookupTransform(config_.base_frame, config_.lidar_frame);
    // initialize data structures
    imu_buffer_.set_capacity(5000);
    ikdtree_ptr_ = KD_TREE<PointType>::Ptr(new KD_TREE<PointType>(config_.ikd_alpha_del, config_.ikd_alpha_bal, config_.ikd_downsample_l, config_.ikd_downsample_n));
    map_pc_buffer_.reserve(300);
    map_pc_eigen_buffer_.reserve(300);
    map_pose_buffer_.reserve(300);
}

/*----------------------
|  CALLBACK FUNCTIONS  |
----------------------*/

void LIONode::processPoints(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // runtime_start
    runtime_t0 = std::chrono::high_resolution_clock::now();
    runtime_pp_t0 = std::chrono::high_resolution_clock::now();
    if (!tf2_buffer_._frameExists(config_.lidar_frame))
    {
        return;
    }
    switch (config_.initial_method)
    {
    case KALMAN:
        if (!tf2_buffer_._frameExists(config_.kalman_odom_frame))
        {
            return;
        };
        if (!tf2_buffer_.canTransform(config_.kalman_odom_frame, config_.base_frame, msg->header.stamp))
        {
            return;
        }
        break;
    case IMU:
        if (!tf2_buffer_._frameExists(config_.imu_odom_frame))
        {
            return;
        };
        if (!tf2_buffer_.canTransform(config_.imu_odom_frame, config_.base_frame, msg->header.stamp))
        {
            return;
        }
        break;
    }
    if (!tf2_buffer_.canTransform(config_.lidar_frame, config_.base_frame, msg->header.stamp))
    {
        return;
    }

    tf_lidar2base_ = LookupTransform(config_.base_frame, config_.lidar_frame, msg->header.stamp);

    auto pc_undist = boost::make_shared<pcl::PointCloud<PointType>>();

    if (config_.deskewing)
    {
        // Deskewing needs timestamp and ring fields -> parse as custom type
        auto pc_raw = boost::make_shared<pcl::PointCloud<custom_type::PointXYZITR>>();
        pcl::fromROSMsg(*msg, *pc_raw);
        pc_raw->header.frame_id = config_.lidar_frame;

        auto pc_filt = boost::make_shared<pcl::PointCloud<custom_type::PointXYZITR>>();
        pc_filt->points.reserve(pc_raw->points.size());
        pc_filt->header = pc_raw->header;
        pc_filt->is_dense = pc_raw->is_dense;
        dist_filter(pc_raw, pc_filt, config_.max_range, config_.min_range);

        pc_undist->points.reserve(pc_filt->points.size());
        pc_undist->header = pc_filt->header;
        pc_undist->is_dense = pc_filt->is_dense;

        // preprocess(pc_filt, pc_undist, msg->header.stamp.toSec());
        if (!undistort(pc_filt, pc_undist, initial_guess))
        {
            return;
        }
    }
    else
    {
        // No deskewing needed -> parse directly as PointXYZ (no timestamp/ring required)
        auto pc_raw = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *pc_raw);
        pc_raw->header.frame_id = config_.lidar_frame;

        auto pc_filt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pc_filt->points.reserve(pc_raw->points.size());
        pc_filt->header = pc_raw->header;
        pc_filt->is_dense = pc_raw->is_dense;
        dist_filter(pc_raw, pc_filt, config_.max_range, config_.min_range);

        pc_undist->points.reserve(pc_filt->points.size());
        pc_undist->header = pc_filt->header;
        pc_undist->is_dense = pc_filt->is_dense;

        for (size_t i = 0; i < pc_filt->points.size(); ++i)
        {
            pc_undist->push_back(pc_filt->points[i]);
        }
    }

    // voxelize points old
    auto pc_vx_map = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_vx_map->points.reserve(pc_undist->points.size());
    pc_vx_map->header = pc_undist->header;
    pc_vx_map->is_dense = pc_undist->is_dense;
    if (config_.map_init_method != OFF && !map_initialized_)
        voxelize(pc_undist, pc_vx_map, config_.map_init_vox_l);
    else
        voxelize(pc_undist, pc_vx_map, config_.vox_l_map);
    auto pc_vx_icp = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_vx_icp->points.reserve(pc_vx_map->points.size());
    pc_vx_icp->header = pc_vx_map->header;
    pc_vx_icp->is_dense = pc_vx_map->is_dense;
    voxelize(pc_vx_map, pc_vx_icp, config_.vox_l_gicp);
    runtime_pp_t1 = std::chrono::high_resolution_clock::now();

    runtime_ig_t0 = std::chrono::high_resolution_clock::now();
    auto [initial_guess, pose_diff] = getInitialGuess(msg->header.stamp);
    runtime_ig_t1 = std::chrono::high_resolution_clock::now();
    if (config_.map_init_method != OFF && !map_initialized_)
    {
        build_map(pc_vx_map, initial_guess, pose_diff, msg->header.stamp);
        return;
    }
    // _______________
    // register points
    Sophus::SE3d registered_pose;

    // apply metric here
    bool reg_suc = false;
    double matching_error = 0;
    runtime_gicp_t0 = std::chrono::high_resolution_clock::now();
    reg_suc = registerPointsGICP(pc_vx_icp, initial_guess, registered_pose, config_.gicp_kernel_value, config_.gicp_max_corr_dist, config_.gicp_max_it, matching_error);
    runtime_gicp_t1 = std::chrono::high_resolution_clock::now();

    // add points to ikd tree
    if (reg_suc)
    {   
        auto pc_vx_map_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pc_vx_map_tf->points.reserve(pc_vx_map->points.size());
        pc_vx_map_tf->header = pc_vx_map->header;
        pc_vx_map_tf->is_dense = pc_vx_map->is_dense;
        pcl::transformPointCloud(*pc_vx_map, *pc_vx_map_tf, registered_pose.translation(), registered_pose.unit_quaternion(), false);

        runtime_map_t0 = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> map_lock(mtx_kdtree_);
            if (ikdtree_ptr_->Root_Node == nullptr)
            {
                ikdtree_ptr_->Build(pc_vx_map_tf->points);
            }
            else
            {
                ikdtree_ptr_->Add_Points(pc_vx_map_tf->points, true);
                int delete_count = 0;
                if (config_.sliding_map)
                {
                    move_map(delete_count, registered_pose);
                }
            }
        }
        runtime_map_t1 = std::chrono::high_resolution_clock::now();

        {
            std::lock_guard<std::mutex> pose_lock(mtx_pose_);
            poses_.push_back(registered_pose);
        }
        const auto model_deviation = initial_guess.inverse() * registered_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
    }
    else
    {
        {
            std::lock_guard<std::mutex> pose_lock(mtx_pose_);
            poses_.push_back(registered_pose);
        }
    }

    {
        std::lock_guard<std::mutex> pose_lock(mtx_pose_);
        if (poses_.empty())
            poses_.push_back(registered_pose);
    }
    publishOdometry(msg->header.stamp);
    if (config_.publish_clouds)
        publishPointClouds(reg_suc, pc_undist, msg->header.stamp, registered_pose);
    if (config_.publish_traj || config_.publish_poses)
        publishPosesTraj(initial_guess, registered_pose, reg_suc, msg->header.stamp);

    runtime_t1 = std::chrono::high_resolution_clock::now();
    evaluation_msgs::Runtime runtime_msg;
    runtime_msg.header.stamp = msg->header.stamp;

    runtime_msg.total = std::chrono::duration<double, std::milli>(runtime_t1 - runtime_t0).count();
    runtime_msg.pp = std::chrono::duration<double, std::milli>(runtime_pp_t1 - runtime_pp_t0).count();
    runtime_msg.ig = std::chrono::duration<double, std::milli>(runtime_ig_t1 - runtime_ig_t0).count();
    runtime_msg.gicp = std::chrono::duration<double, std::milli>(runtime_gicp_t1 - runtime_gicp_t0).count();
    runtime_msg.map = std::chrono::duration<double, std::milli>(runtime_map_t1 - runtime_map_t0).count();

    evaluation_msgs::Error error_msg;
    error_msg.header.stamp = msg->header.stamp;

    error_msg.error = matching_error;
    error_msg.reg_suc = reg_suc;

    evaluation_msgs::Treesize treesize_msg;
    treesize_msg.header.stamp = msg->header.stamp;

    int32_t points_ikd_tree = 0;
    {
        std::lock_guard<std::mutex> map_lock(mtx_kdtree_);
        if (!(ikdtree_ptr_->Root_Node == nullptr))
        {
            points_ikd_tree = ikdtree_ptr_->validnum();
        }
    }
    treesize_msg.num_points = points_ikd_tree;
    treesize_msg.reg_suc = reg_suc;

    runtime_pub_.publish(runtime_msg);
    error_pub_.publish(error_msg);
    treesize_pub_.publish(treesize_msg);
    if (config_.print_runtime)
    {
        ROS_INFO_STREAM("Runtime (ms): " << (std::chrono::duration<double, std::milli>(runtime_t1 - runtime_t0)).count());
        ROS_INFO_STREAM("Runtime pp (ms): " << (std::chrono::duration<double, std::milli>(runtime_pp_t1 - runtime_pp_t0)).count());
        ROS_INFO_STREAM("Runtime ig (ms): " << (std::chrono::duration<double, std::milli>(runtime_ig_t1 - runtime_ig_t0)).count());
        ROS_INFO_STREAM("Runtime gicp (ms): " << (std::chrono::duration<double, std::milli>(runtime_gicp_t1 - runtime_gicp_t0)).count());
        ROS_INFO_STREAM("Runtime map (ms): " << (std::chrono::duration<double, std::milli>(runtime_map_t1 - runtime_map_t0)).count());
    }
}

void LIONode::processIMU(const state_estimator_msgs::EstimatorConstPtr &msg)
{
    imu_meas_.stamp = msg->header.stamp.toSec();
    imu_meas_.ang_vel = tf2::quatRotate(imu2lidar_rot, 
        tf2::Vector3(
            msg->imu.angular_velocity.x, msg->imu.angular_velocity.y, msg->imu.angular_velocity.z)
    );
    imu_meas_.pose = Sophus::SE3d(
        Sophus::SE3d::QuaternionType(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
        Sophus::SE3d::Point(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)
    );
    mtx_imu_.lock();
    imu_buffer_.push_front(this->imu_meas_);
    mtx_imu_.unlock();
    cv_imu_stamp_.notify_one();
}

void LIONode::processPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {}

void LIONode::build_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, 
                        const Sophus::SE3d &pose, 
                        const Sophus::SE3d &pose_diff, 
                        const ros::Time &stamp)
{
    // Integrate incremental yaw (about global Z) from the relative pose
    const Eigen::Vector3d rotvec = pose_diff.so3().log(); // axis-angle (axis * angle), |rotvec| in [0, pi]
    const double dpitch = rotvec.x();
    const double droll = rotvec.y();                      // signed rotation around Z [rad]
    static int count = 0;
    // applying moving_average filter
    d_roll_.push_back(droll);
    d_pitch_.push_back(dpitch);
    if (!map_init_started_)
    {
        if (angle_roll_ == 0)
        {
            if (d_roll_.size() > 5)
            {
                if (d_roll_sum_ == 0)
                    d_roll_sum_ = std::accumulate(d_roll_.begin(), d_roll_.end() - 1, 0);
                if (std::abs(droll - d_roll_sum_ / (d_roll_.size() - 1)) > 0.01)
                    angle_roll_ += droll;
                d_roll_sum_ = d_roll_sum_ - d_roll_.front() + d_roll_.back();
                d_roll_.pop_front();
            }
        }
        else
        {
            map_init_started_ = true;
            return;
        }

        if (angle_pitch_ == 0)
        {
            if (d_pitch_.size() > 5)
            {
                if (d_pitch_sum_ == 0)
                    d_pitch_sum_ = std::accumulate(d_pitch_.begin(), d_pitch_.end() - 1, 0);
                if (std::abs(dpitch - d_pitch_sum_ / (d_pitch_.size() - 1)) > 0.01)
                    angle_pitch_ += dpitch;
                d_pitch_sum_ = d_pitch_sum_ - d_pitch_.front() + d_pitch_.back();
                d_pitch_.pop_front();
            }
        }
        else
        {
            map_init_started_ = true;
            return;
        }
    }
    else
    {
        angle_roll_ += droll;
        angle_pitch_ += dpitch;

        map_pc_buffer_.push_back(*pc_in);
        map_pose_buffer_.push_back(pose);
        std::vector<Eigen::Vector3d> points_tf;
        points_tf.resize((map_pc_buffer_.end() - 1)->size());
        tbb::parallel_for(tbb::blocked_range<size_t>(0, (map_pc_buffer_.end() - 1)->size()),
            [&](const tbb::blocked_range<size_t> &range)
            {
                for (size_t i = range.begin(); i < range.end(); ++i)
                {
                    points_tf[i] = pose * Eigen::Vector3d(
                        (map_pc_buffer_.end() - 1)->points[i].x, (map_pc_buffer_.end() - 1)->points[i].y, (map_pc_buffer_.end() - 1)->points[i].z);
                }
            }
        );
        map_pc_eigen_buffer_.push_back(points_tf);
    }
    
    ROS_INFO_STREAM_THROTTLE(1.0, "-----Init Map Graph (" << config_.map_init_angle << " deg)-----");
    ROS_INFO_STREAM_THROTTLE(1.0, "Roll Angle: " << angle_roll_ / M_PI * 180);
    ROS_INFO_STREAM_THROTTLE(1.0, "Pitch Angle: " << angle_pitch_ / M_PI * 180);

    // Detect completed full turns and keep remainder in angle_
    static int full_turns_roll = 0;
    static int full_turns_pitch = 0;
    static bool init_rotated = false;
    if (std::abs(angle_roll_) >= config_.map_init_angle / 180.0 * M_PI)
    {
        const int dir = (angle_roll_ > 0.0) ? 1 : -1; // +CCW, -CW
        angle_roll_ -= dir * 2.0 * M_PI;              // keep remainder for continued integration
        ++full_turns_roll;
        init_rotated = true;
        ROS_INFO_STREAM("Full self-rotation (roll) detected (" << (dir > 0 ? "+" : "-") << "1), total=" << full_turns_roll);
    }
    if (std::abs(angle_pitch_) >= config_.map_init_angle / 180.0 * M_PI)
    {
        const int dir = (angle_pitch_ > 0.0) ? 1 : -1; // +CCW, -CW
        angle_pitch_ -= dir * 2.0 * M_PI;              // keep remainder for continued integration
        ++full_turns_pitch;
        init_rotated = true;
        ROS_INFO_STREAM("Full self-rotation (pitch) detected (" << (dir > 0 ? "+" : "-") << "1), total=" << full_turns_pitch);
    }
    if (init_rotated)
    {
        int size = map_pose_buffer_.size();
        int k = std::max(1, (int)std::ceil(size / 12.0));
        int i = 0;
        auto pose_it = map_pose_buffer_.begin();
        auto pc_it = map_pc_buffer_.begin();
        std::vector<std::vector<Eigen::Vector3d>> eigen_buffer_reduced;
        std::vector<pcl::PointCloud<PointType>> pc_buffer_reduced;
        std::vector<Sophus::SE3d> pose_buffer_reduced;
        for (auto it = map_pc_eigen_buffer_.begin(); it != map_pc_eigen_buffer_.end(); it++, pose_it++, pc_it++)
        {
            if (i == k)
            {
                i = 0;
            }
            else
            {
                i++;
                continue;
            }
            eigen_buffer_reduced.push_back(*it);
            pc_buffer_reduced.push_back(*pc_it);
            pose_buffer_reduced.push_back(*pose_it);
        }
        if (config_.map_init_method == SLAM)
            performGraphSLAM(eigen_buffer_reduced, pose_buffer_reduced, config_.SLAM_min_corr, config_.SLAM_max_it, config_.SLAM_epsilon, config_.SLAM_max_dist_2);

        for (size_t i = 0; i < pc_buffer_reduced.size(); i++)
        {
            pcl::transformPointCloud(pc_buffer_reduced[i], pc_buffer_reduced[i], pose_buffer_reduced[i].translation(), pose_buffer_reduced[i].unit_quaternion());
            {
                std::lock_guard<std::mutex> map_lock(mtx_kdtree_);
                if (ikdtree_ptr_->Root_Node == nullptr)
                {
                    ikdtree_ptr_->Build(pc_buffer_reduced[i].points); 
                }
                else
                {
                    int num_points = ikdtree_ptr_->Add_Points(pc_buffer_reduced[i].points, true);
                    ROS_INFO_STREAM(num_points << " points added to the ikd-Tree.");
                }
            }
        }
        auto map = boost::make_shared<pcl::PointCloud<PointType>>();
        ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, map->points, NOT_RECORD);
        sensor_msgs::PointCloud2 map_out;
        pcl::toROSMsg(*map, map_out);
        map_out.header.frame_id = config_.odom_frame;
        map_out.header.stamp = stamp;
        map_pub_.publish(map_out);
        Sophus::SE3d zero;
        if (config_.publish_clouds)
            publishPointClouds(true, map, stamp, zero);
        map_initialized_ = true;
    }
}

/*---------------------------
|   PUBLISHING FUNCTIONS    |
---------------------------*/

void LIONode::publishPointClouds(bool reg_suc, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const ros::Time &stamp, Sophus::SE3d &pose)
{

    // current PointCloud
    auto pc_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc_tf->points.reserve(pc->points.size());
    pc_tf->header = pc->header;
    pc_tf->header.frame_id = config_.odom_frame;
    pc_tf->is_dense = pc->is_dense;
    pcl::transformPointCloud(*pc, *pc_tf, pose.translation(), pose.unit_quaternion(), false);
    sensor_msgs::PointCloud2 scan_out;
    pcl::toROSMsg(*pc, scan_out);
    scan_out.header.frame_id = config_.lidar_frame;
    scan_out.header.stamp = stamp;
    pc_pub_.publish(scan_out);
    // current PointCloud only if registered
    if (reg_suc)
    {
        pcl::toROSMsg(*pc, scan_out);
        scan_out.header.frame_id = config_.lidar_frame;
        scan_out.header.stamp = stamp;
        pc_reg_pub_.publish(scan_out);
    }

    // current global map    
    auto map = boost::make_shared<pcl::PointCloud<PointType>>();
    ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, map->points, NOT_RECORD);
    sensor_msgs::PointCloud2 map_out;
    pcl::toROSMsg(*map, map_out);
    map_out.header.frame_id = config_.odom_frame;
    map_out.header.stamp = stamp;
    map_pub_.publish(map_out);
}

void LIONode::publishOdometry(const ros::Time &stamp)
{
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = config_.lidar_frame;
    transform_msg.child_frame_id = config_.odom_frame;
    Sophus::SE3d pose;
    {
        std::lock_guard<std::mutex> pose_lock(mtx_pose_);
        pose = poses_.empty() ? Sophus::SE3d() : poses_.back().inverse();
    }
    transform_msg.transform = sophusToTransform(pose);
    tf2_broadcaster_.sendTransform(transform_msg);
}

void LIONode::publishPosesTraj(const Sophus::SE3d &initial_guess, const Sophus::SE3d &registered_pose, bool reg_suc, const ros::Time &stamp)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = config_.odom_frame;
    if (config_.publish_traj)
    {
        if (reg_suc)
            pose_msg.pose = sophusToPose(registered_pose);
        else
            pose_msg.pose = sophusToPose(initial_guess);
        all_path_msg_.poses.push_back(pose_msg);
        traj_all_pub_.publish(all_path_msg_);

        pose_msg.pose = sophusToPose(registered_pose);
        all_path2_msg_.poses.push_back(pose_msg);
        traj_all2_pub_.publish(all_path2_msg_);

        if (reg_suc)
        {
            pose_msg.pose = sophusToPose(registered_pose);
            reg_path_msg_.poses.push_back(pose_msg);
        }
        traj_reg_pub_.publish(reg_path_msg_);
    }
    if (config_.publish_poses)
    {
        if (reg_suc)
        {
            pose_msg.pose = sophusToPose(registered_pose);
            pose_reg_pub_.publish(pose_msg);
        }
        pose_msg.pose = sophusToPose(registered_pose);
        pose_all_pub_.publish(pose_msg);
        pose_msg.pose = sophusToPose(initial_guess);
        init_pose_pub_.publish(pose_msg);
    }
}
