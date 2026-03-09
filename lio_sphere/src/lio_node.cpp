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
    // ROS_WARN("Constructor");
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

    /*     nh_.param("deskewing", config_.deskewing, config_.deskewing);
    nh_.param("min_range", config_.min_range, config_.min_range);
    nh_.param("max_range", config_.max_range, config_.max_range);
    nh_.param("vox_l_map", config_.vox_l_map, config_.vox_l_map);
    nh_.param("vox_l_gicp", config_.vox_l_gicp, config_.vox_l_gicp);
    if (nh_.hasParam("initial_method")){
    std::string initial_method;
    nh_.getParam("initial_method", initial_method);
    if (initial_method == "KALMAN" || initial_method == "kalman") config_.initial_method = InitialMethod::KALMAN;
    if (initial_method == "IMU" || initial_method == "imu") config_.initial_method = InitialMethod::IMU;
    }
    nh_.param("robust_kernel", config_.robust_kernel, config_.robust_kernel);
    nh_.param("target_NN_k", config_.target_NN_k, config_.target_NN_k);
    nh_.param("gicp_th_fitness", config_.gicp_th_fitness, config_.gicp_th_fitness);
    nh_.param("gicp_th_rmse", config_.gicp_th_rmse, config_.gicp_th_rmse);
    nh_.param("gicp_max_corr_dist", config_.gicp_max_corr_dist, config_.gicp_max_corr_dist);
    nh_.param("adaptive_th", config_.adaptive_th, config_.adaptive_th);
    nh_.param("initial_th", config_.initial_th, config_.initial_th);
    nh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    if (nh_.hasParam("map_init_method")){
    std::string map_init_method;
    nh_.getParam("map_init_method", map_init_method);
    if (map_init_method == "OFF" || map_init_method == "off") config_.map_init_method = MapInitMethod::OFF;
    if (map_init_method == "COMBINE" || map_init_method == "combine") config_.map_init_method = MapInitMethod::COMBINE;
    if (map_init_method == "SLAM" || map_init_method == "slam") config_.map_init_method = MapInitMethod::SLAM;
    }
    nh_.param("map_init_angle", config_.map_init_angle, config_.map_init_angle);
    nh_.param("map_init_vox_l", config_.map_init_vox_l, config_.map_init_vox_l);
    nh_.param("SLAM_max_it", config_.SLAM_max_it, config_.SLAM_max_it);
    nh_.param("SLAM_epsilon", config_.SLAM_epsilon, config_.SLAM_epsilon);
    nh_.param("SLAM_max_dist2", config_.SLAM_max_dist_2, config_.SLAM_max_dist_2);
    nh_.param("SLAM_min_corr", config_.SLAM_min_corr, config_.SLAM_min_corr);
    nh_.param("ikd_alpha_bal", config_.ikd_alpha_bal, config_.ikd_alpha_bal);
    nh_.param("ikd_alpha_del", config_.ikd_alpha_del, config_.ikd_alpha_del);
    nh_.param("ikd_downsample_l", config_.ikd_downsample_l, config_.ikd_downsample_l);
    nh_.param("ikd_downsample_n", config_.ikd_downsample_n, config_.ikd_downsample_n);
    nh_.param("publish_clouds", config_.publish_clouds, config_.publish_clouds);
    nh_.param("publish_traj", config_.publish_traj, config_.publish_traj);
    nh_.param("publish_poses", config_.publish_poses, config_.publish_poses); */

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
    ROS_INFO_STREAM("Got new points! CallbackPTS");

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

    ROS_INFO_STREAM("Configs and Transforms OK!");
    auto pc_raw = boost::make_shared<pcl::PointCloud<custom_type::PointXYZITR>>();
    pcl::fromROSMsg(*msg, *pc_raw);
    tf_lidar2base_ = LookupTransform(config_.base_frame, config_.lidar_frame, msg->header.stamp);
    pcl::transformPointCloud(*pc_raw, *pc_raw, tf_lidar2base_.translation(), tf_lidar2base_.unit_quaternion());
    pc_raw->header.frame_id = config_.base_frame;

    // filter out too close and too far points
    auto pc_filt = boost::make_shared<pcl::PointCloud<custom_type::PointXYZITR>>();
    pc_filt->points.reserve(pc_raw->points.size());
    pc_filt->header = pc_raw->header;
    pc_filt->is_dense = pc_raw->is_dense;
    dist_filter(pc_raw, pc_filt, config_.max_range, config_.min_range);
    auto pc_undist = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_undist->points.reserve(pc_filt->points.size());
    pc_undist->header = pc_filt->header;
    pc_undist->is_dense = pc_filt->is_dense;

    // apply deskewing if enabled
    if (config_.deskewing)
    {
        ROS_INFO_STREAM("Now deskewing the points");
        // preprocess(pc_filt, pc_undist, msg->header.stamp.toSec());
        if (!undistort(pc_filt, pc_undist, initial_guess))
        {
            return;
        }
        ROS_INFO_STREAM("Deskewing success");
    }
    else
    {
        for (size_t i = 0; i < pc_filt->points.size(); ++i)
        {
            pc_undist->push_back(pcl::PointXYZ(pc_filt->points[i].x, pc_filt->points[i].y, pc_filt->points[i].z));
        }
    }

    // only for debugging of the preprocessing
    // auto pc_out = boost::make_shared<pcl::PointCloud<PointType>>();
    // auto pc_xyz = boost::make_shared<pcl::PointCloud<PointType>>();
    // pc_xyz->points.reserve(pc_raw->points.size());
    // pc_out->points.reserve(pc_raw->points.size());
    // for (size_t i = 0; i<pc_raw->points.size(); ++i){
    //     pc_xyz->push_back(pcl::PointXYZ(pc_raw->points[i].x, pc_raw->points[i].y, pc_raw->points[i].z));
    // }
    // voxelize(pc_dist, pc_out, 0.5);
    // auto pose_distorted = LookupTransform(config_.base_frame, prev_odom_frame, msg->header.stamp).inverse();
    // publishPointClouds(pc_out, msg->header.stamp, pose_distorted);
    // return;
    // preprocess(pc_raw, pc_dist, msg->header.stamp.toSec());

    // voxelize points old
    ROS_INFO_STREAM("Now voxelizing the points");
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

    // voxelize point new
    /*auto pc_vx_map = boost::make_shared<pcl::PointCloud<PointType>>();
    *pc_vx_map=*pc_undist;
    //if (config_.map_init_method!=OFF && !map_initialized_) voxelize(pc_undist, pc_vx_map, config_.map_init_vox_l);
    voxelize(pc_undist, pc_vx_map, config_.vox_l_map);
    auto pc_vx_icp = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_vx_icp->points.reserve(pc_vx_map->points.size());
    pc_vx_icp->header = pc_vx_map->header;
    pc_vx_icp->is_dense = pc_vx_map->is_dense;
    voxelize(pc_vx_map, pc_vx_icp, config_.vox_l_gicp);
    runtime_pp_t1 = std::chrono::high_resolution_clock::now();*/

    runtime_ig_t0 = std::chrono::high_resolution_clock::now();
    auto [initial_guess, pose_diff] = getInitialGuess(msg->header.stamp);
    runtime_ig_t1 = std::chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Now initializing the map");
    if (config_.map_init_method != OFF && !map_initialized_)
    {
        build_map(pc_vx_map, initial_guess, pose_diff, msg->header.stamp);
        return;
    }
    // return;
    // ROS_INFO_STREAM("initial_guess: " << initial_guess.translation() << ", " << initial_guess.unit_quaternion().x() <<", " << initial_guess.unit_quaternion().y() << ", " << initial_guess.unit_quaternion().z() <<", " << initial_guess.unit_quaternion().w());

    ROS_INFO_STREAM("Now broadcasting a transform");
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = msg->header.stamp;
    transform_msg.header.frame_id = config_.odom_frame;
    transform_msg.child_frame_id = "initial_guess";
    // auto pose = LookupTransform(config_.base_frame, prev_odom_frame, stamp).inverse();
    transform_msg.transform = sophusToTransform(initial_guess);
    tf2_broadcaster_.sendTransform(transform_msg);

    /*transform_msg.header.stamp = msg->header.stamp;
    transform_msg.header.frame_id = config_.odom_frame;
    transform_msg.child_frame_id = "ekf_pose";
    const auto init_pose = LookupTransform(config_.base_frame, config_.kalman_odom_frame, msg->header.stamp).inverse();
    //auto pose = LookupTransform(config_.base_frame, prev_odom_frame, stamp).inverse();
    transform_msg.transform = sophusToTransform(init_pose);
    tf2_broadcaster_.sendTransform(transform_msg);
    */

    // OVERLAP//

    /* auto pc_overlap = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_overlap->points.reserve(pc_filt->points.size());
    pc_filt->header = pc_filt->header;
    pc_filt->is_dense = pc_filt->is_dense;
    voxelize(pc_vx_icp, pc_overlap, config_.overlap_map_voxel_size);
    std::vector<Eigen::Vector3d> pc_overlap_eigen(pc_overlap->points.size());
    for (size_t i = 0; i < pc_overlap->points.size(); i++){
    pc_overlap_eigen[i]=Eigen::Vector3d((double)pc_overlap->points[i].x, (double)pc_overlap->points[i].y, (double)pc_overlap->points[i].z);
    }
    int coverage = overlap_map_.Overlap(pc_overlap_eigen, initial_guess);
    double rel_coverage = (double)coverage/pc_overlap_eigen.size();
    */
    // overlap should be > 0.9
    // ROS_INFO_STREAM("Overlap: " << rel_coverage*100 << " %");
    // compare_transformation(pc_overlap, pc_overlap_eigen, initial_guess);

    // _____________
    // DEBUGGING
    // auto pc_vx_map_tf_debug = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pc_vx_map_tf_debug->points.reserve(pc_vx_icp->points.size());
    // pc_vx_map_tf_debug->header = pc_vx_icp->header;
    // pc_vx_map_tf_debug->header.frame_id = config_.odom_frame;
    // pc_vx_map_tf_debug->is_dense = pc_vx_icp->is_dense;
    // pcl::transformPointCloud(*pc_vx_icp, *pc_vx_map_tf_debug, initial_guess.translation(), initial_guess.unit_quaternion(), false);
    // sensor_msgs::PointCloud2 scan_out;
    // pcl::toROSMsg(*pc_vx_map_tf_debug, scan_out);
    // scan_out.header.frame_id = config_.odom_frame;
    // scan_out.header.stamp = msg->header.stamp;
    // pc_debug_pub_.publish(scan_out);
    // _______________
    ROS_INFO_STREAM("Now registering the points with GICP");
    // register points
    Sophus::SE3d registered_pose;
    // TODO:: method for determining the initial_pose DONE
    // TODO:: motion threshold and kernel DONE
    const double sigma = 0; // GetAdaptiveThreshold();
    // ROS_INFO_STREAM("sigma: " << sigma);

    // apply metric here
    //(rel_coverage<0.01 || rel_coverage>rel_cov_th_) &&
    bool reg_suc = false;
    no_match = false;
    double matching_error = 0;
    // ROS_WARN("max_num_iterations_set: %d", config_.gicp_max_it);
    runtime_gicp_t0 = std::chrono::high_resolution_clock::now();
    reg_suc = registerPointsGICP(pc_vx_icp, initial_guess, registered_pose, config_.gicp_kernel_value, config_.gicp_max_corr_dist, config_.gicp_max_it, sigma, matching_error);
    runtime_gicp_t1 = std::chrono::high_resolution_clock::now();
    // if (2*acos((initial_guess.inverse()*registered_pose).unit_quaternion().w())>M_PI/32){ROS_WARN_STREAM("deviation too big"); registered_pose = initial_guess;}
    // add points to ikd tree
    if (reg_suc)
    {   
        ROS_INFO_STREAM("GICP Registration success!");
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
                    // ROS_WARN_STREAM("map moved, points deleted: " << delete_count);
                }
                // OVERLAP//
                // overlap_map_.Update(pc_overlap_eigen, registered_pose);
            }
        }
        runtime_map_t1 = std::chrono::high_resolution_clock::now();

        // transform_msg.header.stamp = msg->header.stamp;
        // transform_msg.header.frame_id = config_.odom_frame;
        // transform_msg.child_frame_id = "registered_pose";
        // //auto pose = LookupTransform(config_.base_frame, prev_odom_frame, stamp).inverse();
        // transform_msg.transform = sophusToTransform(registered_pose);
        // tf2_broadcaster_.sendTransform(transform_msg);

        {
            std::lock_guard<std::mutex> pose_lock(mtx_pose_);
            poses_.push_back(registered_pose);
        }
        const auto model_deviation = initial_guess.inverse() * registered_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
        // ROS_INFO_STREAM("update trans: " << model_deviation.translation().x() <<", " << model_deviation.translation().y()<<", " << model_deviation.translation().z());
        // ROS_INFO_STREAM("update rot: " << model_deviation.unit_quaternion().x() <<", "<<model_deviation.unit_quaternion().y() <<", "<<model_deviation.unit_quaternion().z() <<", "<<model_deviation.unit_quaternion().w());
        // //no_match_counter_ = 0;
        // rel_cov_th_ = 0.9;
        // max_distance_ = 3.0;
        // ROS_INFO_STREAM("reg_pose " << registered_pose.translation() << " , " << registered_pose.unit_quaternion().x()<< ", " << registered_pose.unit_quaternion().y()<<", " << registered_pose.unit_quaternion().z()<<", " << registered_pose.unit_quaternion().w());
    }
    else
    {
        ROS_INFO_STREAM("GICP registration FAILED!");
        {
            std::lock_guard<std::mutex> pose_lock(mtx_pose_);
            poses_.push_back(registered_pose);
        }
        // no_match = true;
        // ROS_INFO_STREAM("error");
        // no_match_counter_ ++;
        // registered_pose = initial_guess;
        rel_cov_th_ -= 0.05;
        max_distance_ += 0.2;
        // ROS_INFO_STREAM("init_pose " << registered_pose.translation() << " , " << registered_pose.unit_quaternion().x()<< ", " << registered_pose.unit_quaternion().y()<<", " << registered_pose.unit_quaternion().z()<<", " << registered_pose.unit_quaternion().w());
    }
    // if (!(ikdtree_ptr_->Root_Node == nullptr)){
    //         ROS_INFO_STREAM("Tree size: " << ikdtree_ptr_->validnum());
    // }
    // poses_.push_back(registered_pose);
    //  _____________
    //  DEBUGGING
    //  auto pc_vx_map_tf_debug = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    //  pc_vx_map_tf_debug->points.reserve(pc_vx_icp->points.size());
    //  pc_vx_map_tf_debug->header = pc_vx_icp->header;
    //  pc_vx_map_tf_debug->header.frame_id = config_.odom_frame;
    //  pc_vx_map_tf_debug->is_dense = pc_vx_icp->is_dense;
    //  pcl::transformPointCloud(*pc_vx_icp, *pc_vx_map_tf_debug, initial_guess.translation(), initial_guess.unit_quaternion(), false);
    //  sensor_msgs::PointCloud2 scan_out;
    //  pcl::toROSMsg(*pc_vx_map_tf_debug, scan_out);
    //  scan_out.header.frame_id = config_.odom_frame;
    //  scan_out.header.stamp = msg->header.stamp;
    //  pc_debug_pub_.publish(scan_out);
    ROS_INFO_STREAM("Publish odom");
    //  _______________
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
            ROS_INFO_STREAM("Tree size: " << points_ikd_tree);
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
    imu_meas_.ang_vel = tf2::quatRotate(imu2lidar_rot, tf2::Vector3(msg->imu.angular_velocity.x, msg->imu.angular_velocity.y, msg->imu.angular_velocity.z));
    imu_meas_.pose = Sophus::SE3d(Sophus::SE3d::QuaternionType(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                  Sophus::SE3d::Point(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    mtx_imu_.lock();
    imu_buffer_.push_front(this->imu_meas_);
    mtx_imu_.unlock();
    cv_imu_stamp_.notify_one();
    // DEBUGGING
    imu_x_ += msg->imu.angular_velocity.x;
    imu_y_ += msg->imu.angular_velocity.y;
    imu_z_ += msg->imu.angular_velocity.z;
    imu_num_ += 1;
}
void LIONode::processPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {}
void LIONode::build_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, const Sophus::SE3d &pose, const Sophus::SE3d &pose_diff, const ros::Time &stamp)
{
    // Integrate incremental yaw (about global Z) from the relative pose
    const Eigen::Vector3d rotvec = pose_diff.so3().log(); // axis-angle (axis * angle), |rotvec| in [0, pi]
    const double dpitch = rotvec.x();
    const double droll = rotvec.y();                      // signed rotation around Z [rad]
    //const double dyaw = rotvec.z();
    static int count = 0;
    // applying moving_average filter
    d_roll_.push_back(droll);
    d_pitch_.push_back(dpitch);
    //d_yaw_.push_back(dyaw);
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
        // if (angle_yaw_ == 0)
        // {
        //     if (d_yaw_.size() > 5)
        //     {
        //         if (d_yaw_sum_ == 0)
        //             d_yaw_sum_ = std::accumulate(d_yaw_.begin(), d_yaw_.end() - 1, 0);
        //         if (std::abs(dyaw - d_yaw_sum_ / (d_yaw_.size() - 1)) > 0.01)
        //             angle_yaw_ += dyaw;
        //         d_yaw_sum_ = d_yaw_sum_ - d_yaw_.front() + d_yaw_.back();
        //         d_yaw_.pop_front();
        //     }
        // }
        // else
        // {
        //     map_init_started_ = true;
        //     return;
        // }
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
        // if (angle_roll_ != 0) angle_roll_ += droll;
        // if (angle_yaw_ != 0) angle_yaw_ += dyaw;
        // if (angle_pitch_ != 0) angle_pitch_ += dpitch;
        angle_roll_ += droll;
        //angle_yaw_ += dyaw;
        angle_pitch_ += dpitch;

        map_pc_buffer_.push_back(*pc_in);
        // pcl::transformPointCloud(*(map_pc_buffer_.end()-1), *(map_pc_buffer_.end()-1), pose.translation(), pose.unit_quaternion());
        map_pose_buffer_.push_back(pose);
        std::vector<Eigen::Vector3d> points_tf;
        points_tf.resize((map_pc_buffer_.end() - 1)->size());
        tbb::parallel_for(tbb::blocked_range<size_t>(0, (map_pc_buffer_.end() - 1)->size()),
                          [&](const tbb::blocked_range<size_t> &range)
                          {
                              for (size_t i = range.begin(); i < range.end(); ++i)
                              {
                                  points_tf[i] = pose * Eigen::Vector3d((map_pc_buffer_.end() - 1)->points[i].x, (map_pc_buffer_.end() - 1)->points[i].y, (map_pc_buffer_.end() - 1)->points[i].z);
                              }
                          });
        // for (size_t i = 0; i < (map_pc_buffer_.end()-1)->size(); i++){
        //     points_tf[i]=Eigen::Vector3d((map_pc_buffer_.end()-1)->points[i].x, (map_pc_buffer_.end()-1)->points[i].y, (map_pc_buffer_.end()-1)->points[i].z);
        // }
        map_pc_eigen_buffer_.push_back(points_tf);
    }
    // add functionality for pitch and yaw angle

    // angle_roll_ += droll;
    //  angle_pitch_ += dpitch;
    //  angle_yaw_ += dyaw;
    // ROS_INFO_STREAM("Angle: " << angle_roll_/M_PI*180);

    ROS_INFO_STREAM_THROTTLE(1.0, "-----Init Map Graph (" << config_.map_init_angle << "°) --------");
    ROS_INFO_STREAM_THROTTLE(1.0, "Roll Angle: " << angle_roll_ / M_PI * 180);
    ROS_INFO_STREAM_THROTTLE(1.0, "Pitch Angle: " << angle_pitch_ / M_PI * 180);
    //ROS_INFO_STREAM_THROTTLE(1.0, "Yaw Angle: " << angle_yaw_ / M_PI * 180);

    // Detect completed full turns and keep remainder in angle_
    static int full_turns_roll = 0;
    static int full_turns_pitch = 0;
    //static int full_turns_yaw = 0;
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
    // if (std::abs(angle_yaw_) >= config_.map_init_angle / 180.0 * M_PI)
    // {
    //     const int dir = (angle_yaw_ > 0.0) ? 1 : -1; // +CCW, -CW
    //     angle_yaw_ -= dir * 2.0 * M_PI;              // keep remainder for continued integration
    //     ++full_turns_yaw;
    //     init_rotated = true;
    //     ROS_INFO_STREAM("Full self-rotation (yaw) detected (" << (dir > 0 ? "+" : "-") << "1), total=" << full_turns_yaw);
    // }
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
            // pcl::transformPointCloud(*it, *it, pose_it->translation(), pose_it->unit_quaternion());
            if (i == k)
            {
                i = 0;
            }
            else
            {
                i++;
                continue;
            }
            ROS_INFO_STREAM("call");
            eigen_buffer_reduced.push_back(*it);
            pc_buffer_reduced.push_back(*pc_it);
            pose_buffer_reduced.push_back(*pose_it);
        }
        ROS_INFO_STREAM("initial: " << eigen_buffer_reduced.size());
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
// ROS_INFO_STREAM("Angle: " << angle_/M_PI*180);
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
    // pcl::toROSMsg(*pc_tf, scan_out);
    pcl::toROSMsg(*pc, scan_out);
    // scan_out.header.frame_id = config_.odom_frame;
    scan_out.header.frame_id = config_.base_frame;
    scan_out.header.stamp = stamp;
    pc_pub_.publish(scan_out);
    // current PointCloud only if registered
    if (reg_suc)
    {
        // pcl::toROSMsg(*pc_tf, scan_out);
        pcl::toROSMsg(*pc, scan_out);
        // scan_out.header.frame_id = config_.odom_frame;
        scan_out.header.frame_id = config_.base_frame;
        scan_out.header.stamp = stamp;
        pc_reg_pub_.publish(scan_out);
    }
    // pc_pub_.publish(scan_out);
    // sensor_msgs::PointCloud2 pc_msg;
    // pcl::toROSMsg(*pc, pc_msg);
    // pc_msg.header.frame_id = config_.odom_frame;
    // pc_msg.header.stamp = stamp;
    // const auto base2odom = LookupTransform(prev_odom_frame_, config_.base_frame_, stamp);
    // pcl::transformPointCloud(*pc, *pc, base2odom.translation(), base2odom.unit_quaternion());
    // pc->header.frame_id = config_.odom_frame;
    // sensor_msgs::PointCloud2 scan_out;
    // pcl::toROSMsg(*pc, scan_out);
    // scan_out.header.frame_id = config_.odom_frame;
    // scan_out.header.stamp = stamp;
    // pc_pub_.publish(scan_out);

    // current map
    /*
    auto map = boost::make_shared<pcl::PointCloud<PointType>>();
    ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, map->points, NOT_RECORD);
    sensor_msgs::PointCloud2 map_out;
    pcl::toROSMsg(*map, map_out);
    map_out.header.frame_id = config_.odom_frame;
    map_out.header.stamp = stamp;
    map_pub_.publish(map_out);*/
}
void LIONode::publishOdometry(const ros::Time &stamp)
{
    ROS_INFO_STREAM("Call publishOdometry() from " << config_.base_frame << " to " << config_.odom_frame);
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = config_.base_frame;
    transform_msg.child_frame_id = config_.odom_frame;
    Sophus::SE3d pose;
    {
        std::lock_guard<std::mutex> pose_lock(mtx_pose_);
        pose = poses_.empty() ? Sophus::SE3d() : poses_.back().inverse(); // LookupTransform(config_.base_frame, prev_odom_frame, stamp);
    }
    transform_msg.transform = sophusToTransform(pose);
    tf2_broadcaster_.sendTransform(transform_msg);
}
void LIONode::publishPosesTraj(const Sophus::SE3d &initial_guess, const Sophus::SE3d &registered_pose, bool reg_suc, const ros::Time &stamp)
{
    ROS_INFO_STREAM("publishPosesTraj() called");
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
        ROS_INFO_STREAM("now publishing pose_all_pub_ and init_pose_pub_");
        pose_msg.pose = sophusToPose(registered_pose);
        pose_all_pub_.publish(pose_msg);
        pose_msg.pose = sophusToPose(initial_guess);
        init_pose_pub_.publish(pose_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh;       // public namespace
    ros::NodeHandle pnh("~"); // private namespace
    LIONode node(nh, pnh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

// DEBUGGING
void LIONode::preprocess(pcl::PointCloud<custom_type::PointXYZITR>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, double start_time)
{
    // imu2lidar_tf = lookupTransform("pandar_frame", "imu_frame");
    // calculate LiDAR average rotation velocity
    // tf2::Vector3 om = imu2lidar_tf * tf2::Vector3(imu_x_/imu_num_, imu_y_/imu_num_, imu_z_/imu_num_);
    // tf2::Vector3 om = tf2::quatRotate(imu2lidar_rot, tf2::Vector3(imu_x_/imu_num_, imu_y_/imu_num_, imu_z_/imu_num_));
    tf2::Vector3 om = tf2::Vector3(imu_x_ / imu_num_, imu_y_ / imu_num_, imu_z_ / imu_num_);
    imu_x_ = 0;
    imu_y_ = 0;
    imu_z_ = 0;
    imu_num_ = 0;

    for (size_t i = 0; i < pc_in->points.size(); ++i)
    {
        double point_x = pc_in->points[i].x;
        double point_y = pc_in->points[i].y;
        double point_z = pc_in->points[i].z;
        // std::cout << pc_in->points[i].timestamp-start_time <<std::endl;
        // std::cout << "om: " << om.x() << om.y() << om.z() << std::endl;
        tf2::Vector3 rot = om * (pc_in->points[i].timestamp - start_time); // in local frame
        // std::cout << rot.x << " " << rot.y() << " " << rot.z() << std::endl;
        pcl::PointXYZ point_corr;
        point_corr.x = (cos(rot.y()) * cos(rot.z())) * point_x + (sin(rot.x()) * sin(rot.y()) * cos(rot.z()) - cos(rot.x()) * sin(rot.z())) * point_y + (cos(rot.x()) * sin(rot.y()) * cos(rot.z()) + sin(rot.x()) * sin(rot.z())) * point_z;
        point_corr.y = (cos(rot.y()) * sin(rot.z())) * point_x + (sin(rot.x()) * sin(rot.y()) * sin(rot.z()) + cos(rot.x()) * cos(rot.z())) * point_y + (cos(rot.x()) * sin(rot.y()) * sin(rot.z()) - sin(rot.x()) * cos(rot.z())) * point_z;
        point_corr.z = (-sin(rot.y())) * point_x + (sin(rot.x()) * cos(rot.y())) * point_y + (cos(rot.x()) * cos(rot.y())) * point_z;

        pc_out->push_back(point_corr);
    }
}