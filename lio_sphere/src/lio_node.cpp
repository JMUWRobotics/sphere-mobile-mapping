#include "lio_node.hpp"
#include "graph_slam.hpp"
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

/*-------------------
|   INITIALIZATION  |
-------------------*/

LIONode::LIONode(const ros::NodeHandle& nh) : nh_(nh), tf2_listener_(tf2_buffer_){
//get parameters
    nh_.param("base_frame", base_frame_, base_frame_);
    nh_.param("odom_frame", odom_frame_, odom_frame_);
    nh_.param("visualize", publish_clouds_, publish_clouds_);
    nh_.param("max_range", config_.max_range, config_.max_range);
    nh_.param("min_range", config_.min_range, config_.min_range);
    nh_.param("deskew", config_.deskew, config_.deskew);
    nh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    nh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    nh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
//subscriptions
    pc_sub_ = nh_.subscribe("/points_in", 10, &LIONode::processPoints, this, ros::TransportHints().tcpNoDelay());
    imu_sub_ = nh_.subscribe("/imu_in", 10, &LIONode::processIMU, this, ros::TransportHints().tcpNoDelay());
    pose_sub_ = nh_.subscribe("/lkf/pose", 10, &LIONode::processPose, this, ros::TransportHints().tcpNoDelay());
//publishers
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_out", 1);
    pc_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_debug_out", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_out", 1);
    // GICP debug clouds
    source_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gicp_source", 1);
    target_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gicp_target", 1);
//store static transforms
    tf_imu2lidar_ = LookupTransform(lidar_frame_, imu_frame_);
    tf_lidar2base_ = LookupTransform(base_frame_, lidar_frame_);
//initialize data structures
    imu_buffer_.set_capacity(5000);
    ikdtree_ptr_ = KD_TREE<PointType>::Ptr(new KD_TREE<PointType>(0.3, 0.6, 0.5));
    map_pc_buffer_.reserve(300);
    map_pc_eigen_buffer_.reserve(300);
    map_pose_buffer_.reserve(300);

}

/*----------------------
|  CALLBACK FUNCTIONS  |
----------------------*/

void LIONode::processPoints(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (!tf2_buffer_._frameExists(lidar_frame_)){return;}
    if (!tf2_buffer_._frameExists(prev_odom_frame_)){return;}
    if (!tf2_buffer_.canTransform(prev_odom_frame_, base_frame_, msg->header.stamp)){return;}
    if (!tf2_buffer_.canTransform(lidar_frame_, base_frame_, msg->header.stamp)){return;}
    auto pc_raw = boost::make_shared<pcl::PointCloud<custom_type::PointXYZITR>>();
    pcl::fromROSMsg(*msg, *pc_raw);
    tf_lidar2base_ = LookupTransform(base_frame_, lidar_frame_, msg->header.stamp);
    //TODO eigentlich transformieren in base_frame, nicht imu_frame
    pcl::transformPointCloud(*pc_raw, *pc_raw, tf_lidar2base_.translation(), tf_lidar2base_.unit_quaternion());
    pc_raw->header.frame_id = base_frame_;
    auto pc_dist = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_dist->points.reserve(pc_raw->points.size());
    pc_dist->header = pc_raw->header;
    pc_dist->is_dense = pc_raw->is_dense;
    
    //preprocess(pc_raw, pc_dist, msg->header.stamp.toSec());
    if (!distort(pc_raw, pc_dist, initial_guess)) {return;}
    // only for debugging of the preprocessing
    // auto pc_out = boost::make_shared<pcl::PointCloud<PointType>>();
    // auto pc_xyz = boost::make_shared<pcl::PointCloud<PointType>>();
    // pc_xyz->points.reserve(pc_raw->points.size());
    // pc_out->points.reserve(pc_raw->points.size());
    // for (size_t i = 0; i<pc_raw->points.size(); ++i){
    //     pc_xyz->push_back(pcl::PointXYZ(pc_raw->points[i].x, pc_raw->points[i].y, pc_raw->points[i].z));
    // }
    // voxelize(pc_dist, pc_out, 0.5);
    // auto pose_distorted = LookupTransform(base_frame_, prev_odom_frame_, msg->header.stamp).inverse();
    // publishPointClouds(pc_out, msg->header.stamp, pose_distorted);
    // return;
    //preprocess(pc_raw, pc_dist, msg->header.stamp.toSec());

    //filter out too close and too far points
    auto pc_filt = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_filt->points.reserve(pc_raw->points.size());
    pc_filt->header = pc_raw->header;
    pc_filt->is_dense = pc_raw->is_dense;
    dist_filter(pc_dist, pc_filt, config_.max_range, config_.min_range);

    //voxelize points
    auto pc_vx_map = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_vx_map->points.reserve(pc_filt->points.size());
    pc_vx_map->header = pc_filt->header;
    pc_vx_map->is_dense = pc_filt->is_dense;
    if (!map_initialized_) voxelize(pc_filt, pc_vx_map, 1.0 * config_.voxel_size);
    else voxelize(pc_filt, pc_vx_map, 0.5 * config_.voxel_size);
    auto pc_vx_icp = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_vx_icp->points.reserve(pc_vx_map->points.size());
    pc_vx_icp->header = pc_vx_map->header;
    pc_vx_icp->is_dense = pc_vx_map->is_dense;
    voxelize(pc_vx_map, pc_vx_icp, 1.0 * config_.voxel_size);
    
    //estimate overlap
    //set initial_guess here when it should be obtained from tf topic
    auto [initial_guess, pose_diff] = getInitialGuess(msg->header.stamp);
    if(!map_initialized_){
        build_map(pc_vx_map, initial_guess, pose_diff, msg->header.stamp);
        return;
    }
    //return;
    //ROS_INFO_STREAM("initial_guess: " << initial_guess.translation() << ", " << initial_guess.unit_quaternion().x() <<", " << initial_guess.unit_quaternion().y() << ", " << initial_guess.unit_quaternion().z() <<", " << initial_guess.unit_quaternion().w());
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = msg->header.stamp;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = "initial_guess";
    //auto pose = LookupTransform(base_frame_, prev_odom_frame_, stamp).inverse();
    transform_msg.transform = sophusToTransform(initial_guess);
    tf2_broadcaster_.sendTransform(transform_msg);

    transform_msg.header.stamp = msg->header.stamp;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = "ekf_pose";
    const auto init_pose = LookupTransform(base_frame_, prev_odom_frame_, msg->header.stamp).inverse();
    //auto pose = LookupTransform(base_frame_, prev_odom_frame_, stamp).inverse();
    transform_msg.transform = sophusToTransform(init_pose);
    tf2_broadcaster_.sendTransform(transform_msg);
    
    auto pc_overlap = boost::make_shared<pcl::PointCloud<PointType>>();
    pc_overlap->points.reserve(pc_filt->points.size());
    pc_filt->header = pc_filt->header;
    pc_filt->is_dense = pc_filt->is_dense;
    voxelize(pc_vx_icp, pc_overlap, config_.overlap_map_voxel_size);
    std::vector<Eigen::Vector3d> pc_overlap_eigen(pc_overlap->points.size());
    for (size_t i = 0; i < pc_overlap->points.size(); i++){
        pc_overlap_eigen[i]=Eigen::Vector3d((double)pc_overlap->points[i].x, (double)pc_overlap->points[i].y, (double)pc_overlap->points[i].z);
    }
    int coverage = overlap_map_.Overlap(pc_overlap_eigen, initial_guess);
    double rel_coverage = (double)coverage/pc_overlap_eigen.size(); //overlap should be > 0.9
    //ROS_INFO_STREAM("Overlap: " << rel_coverage*100 << " %");
    //compare_transformation(pc_overlap, pc_overlap_eigen, initial_guess);
    





        // _____________
    // DEBUGGING
    auto pc_vx_map_tf_debug = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc_vx_map_tf_debug->points.reserve(pc_vx_icp->points.size());
    pc_vx_map_tf_debug->header = pc_vx_icp->header;
    pc_vx_map_tf_debug->header.frame_id = odom_frame_;
    pc_vx_map_tf_debug->is_dense = pc_vx_icp->is_dense;
    pcl::transformPointCloud(*pc_vx_icp, *pc_vx_map_tf_debug, initial_guess.translation(), initial_guess.unit_quaternion(), false);
    sensor_msgs::PointCloud2 scan_out;
    pcl::toROSMsg(*pc_vx_map_tf_debug, scan_out);
    scan_out.header.frame_id = odom_frame_;
    scan_out.header.stamp = msg->header.stamp;
    pc_debug_pub_.publish(scan_out);
    // _______________

    //register points
    Sophus::SE3d registered_pose;
    //TODO:: method for determining the initial_pose DONE
    //TODO:: motion threshold and kernel DONE
    const double sigma = GetAdaptiveThreshold();
    
    //apply metric here
    //(rel_coverage<0.01 || rel_coverage>rel_cov_th_) && 
    bool max_it_reached = false;
     if (registerPointsGICP(pc_vx_icp, initial_guess, registered_pose, max_it_reached, max_distance_, 30, sigma)){
        if (2*acos((initial_guess.inverse()*registered_pose).unit_quaternion().w())>M_PI/32){ROS_WARN_STREAM("deviation too big"); registered_pose = initial_guess;}
        //add points to ikd tree
        auto pc_vx_map_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pc_vx_map_tf->points.reserve(pc_vx_map->points.size());
        pc_vx_map_tf->header = pc_vx_map->header;
        pc_vx_map_tf->is_dense = pc_vx_map->is_dense;
        pcl::transformPointCloud(*pc_vx_map, *pc_vx_map_tf, registered_pose.translation(), registered_pose.unit_quaternion(), false);

        if (ikdtree_ptr_->Root_Node == nullptr){
            ikdtree_ptr_->Build(pc_vx_map_tf->points);
        }
        else{
            ikdtree_ptr_->Add_Points(pc_vx_map_tf->points, true);
            int delete_count = 0;
            //move_map(delete_count, registered_pose);
            overlap_map_.Update(pc_overlap_eigen, registered_pose);
        }

        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = "registered_pose";
        //auto pose = LookupTransform(base_frame_, prev_odom_frame_, stamp).inverse();
        transform_msg.transform = sophusToTransform(registered_pose);
        tf2_broadcaster_.sendTransform(transform_msg);
 
        poses_.push_back(registered_pose);
        const auto model_deviation = initial_guess.inverse() * registered_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
        no_match_counter_ = 0;
        rel_cov_th_ = 0.9;
        max_distance_ = 3.0;
        //ROS_INFO_STREAM("reg_pose " << registered_pose.translation() << " , " << registered_pose.unit_quaternion().x()<< ", " << registered_pose.unit_quaternion().y()<<", " << registered_pose.unit_quaternion().z()<<", " << registered_pose.unit_quaternion().w());
    }
    else{
        poses_.push_back(registered_pose);
        //ROS_INFO_STREAM("error");
        //no_match_counter_ ++;
        //registered_pose = initial_guess;
        rel_cov_th_ -= 0.05;
        max_distance_ += 0.2;
        //ROS_INFO_STREAM("init_pose " << registered_pose.translation() << " , " << registered_pose.unit_quaternion().x()<< ", " << registered_pose.unit_quaternion().y()<<", " << registered_pose.unit_quaternion().z()<<", " << registered_pose.unit_quaternion().w());
    }
    if (!(ikdtree_ptr_->Root_Node == nullptr)){
            ROS_INFO_STREAM("Tree size: " << ikdtree_ptr_->validnum());
    }
    //poses_.push_back(registered_pose);
    // _____________
    // DEBUGGING
    // auto pc_vx_map_tf_debug = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pc_vx_map_tf_debug->points.reserve(pc_vx_icp->points.size());
    // pc_vx_map_tf_debug->header = pc_vx_icp->header;
    // pc_vx_map_tf_debug->header.frame_id = odom_frame_;
    // pc_vx_map_tf_debug->is_dense = pc_vx_icp->is_dense;
    // pcl::transformPointCloud(*pc_vx_icp, *pc_vx_map_tf_debug, initial_guess.translation(), initial_guess.unit_quaternion(), false);
    // sensor_msgs::PointCloud2 scan_out;
    // pcl::toROSMsg(*pc_vx_map_tf_debug, scan_out);
    // scan_out.header.frame_id = odom_frame_;
    // scan_out.header.stamp = msg->header.stamp;
    // pc_debug_pub_.publish(scan_out);
    // _______________
    if (poses_.empty()) poses_.push_back(registered_pose);
    publishOdometry(msg->header.stamp);
    publishPointClouds(pc_vx_icp, msg->header.stamp, registered_pose);

}
void LIONode::processIMU(const state_estimator_msgs::EstimatorConstPtr& msg) {
    imu_meas_.stamp = msg->header.stamp.toSec();
    imu_meas_.ang_vel = tf2::quatRotate(imu2lidar_rot, tf2::Vector3(msg->imu.angular_velocity.x, msg->imu.angular_velocity.y, msg->imu.angular_velocity.z));
    imu_meas_.pose = Sophus::SE3d(Sophus::SE3d::QuaternionType(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
        Sophus::SE3d::Point(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    mtx_imu_.lock();
    imu_buffer_.push_front(this->imu_meas_);
    mtx_imu_.unlock();
    cv_imu_stamp_.notify_one();
    //DEBUGGING
    imu_x_ += msg->imu.angular_velocity.x;
    imu_y_ += msg->imu.angular_velocity.y;
    imu_z_ += msg->imu.angular_velocity.z;
    imu_num_ += 1;
}
void LIONode::processPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {}
void LIONode::build_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in, const Sophus::SE3d &pose, const Sophus::SE3d &pose_diff, const ros::Time &stamp){
        // Integrate incremental yaw (about global Z) from the relative pose
    const Eigen::Vector3d rotvec = pose_diff.so3().log();   // axis-angle (axis * angle), |rotvec| in [0, pi]
    const double droll = rotvec.x();                         // signed rotation around Z [rad]
    const double dpitch = rotvec.y();
    const double dyaw = rotvec.z();
    static int count = 0;
    //applying moving_average filter
    d_roll_.push_back(droll);
    d_pitch_.push_back(dpitch);
    d_yaw_.push_back(dyaw);
    if (!map_init_started_){
        if (angle_roll_ == 0){
            if (d_roll_.size() > 5){
                if (d_roll_sum_ == 0) d_roll_sum_ = std::accumulate(d_roll_.begin(), d_roll_.end()-1, 0);
                if (std::abs(droll-d_roll_sum_/(d_roll_.size()-1)) > 0.01) angle_roll_ += droll;
                d_roll_sum_=d_roll_sum_-d_roll_.front()+d_roll_.back();
                d_roll_.pop_front();
            }
        }
        else {map_init_started_=true; return;}
        if (angle_yaw_ == 0){
            if (d_yaw_.size() > 5){
                if (d_yaw_sum_ == 0) d_yaw_sum_ = std::accumulate(d_yaw_.begin(), d_yaw_.end()-1, 0);
                if (std::abs(dyaw-d_yaw_sum_/(d_yaw_.size()-1)) > 0.01) angle_yaw_ += dyaw;
                d_yaw_sum_=d_yaw_sum_-d_yaw_.front()+d_yaw_.back();
                d_yaw_.pop_front();
            }
        }
        else {map_init_started_=true; return;}
        if (angle_pitch_ == 0){
            if (d_pitch_.size() > 5){
                if (d_pitch_sum_ == 0) d_pitch_sum_ = std::accumulate(d_pitch_.begin(), d_pitch_.end()-1, 0);
                if (std::abs(dpitch-d_pitch_sum_/(d_pitch_.size()-1)) > 0.01) angle_pitch_ += dpitch;
                d_pitch_sum_=d_pitch_sum_-d_pitch_.front()+d_pitch_.back();
                d_pitch_.pop_front();
            }
        }
        else {map_init_started_=true; return;}
    }
    else {
        if (angle_roll_ != 0) angle_roll_ += droll; 
        if (angle_yaw_ != 0) angle_yaw_ += dyaw;
        if (angle_pitch_ != 0) angle_pitch_ += dpitch;
        map_pc_buffer_.push_back(*pc_in); 
        //pcl::transformPointCloud(*(map_pc_buffer_.end()-1), *(map_pc_buffer_.end()-1), pose.translation(), pose.unit_quaternion()); 
        map_pose_buffer_.push_back(pose);
        std::vector<Eigen::Vector3d> points_tf;
        points_tf.resize((map_pc_buffer_.end()-1)->size());
        tbb::parallel_for(tbb::blocked_range<size_t>(0, (map_pc_buffer_.end()-1)->size()),
                [&](const tbb::blocked_range<size_t> &range){
                    for (size_t i = range.begin(); i < range.end(); ++i) {
                        points_tf[i]=pose*Eigen::Vector3d((map_pc_buffer_.end()-1)->points[i].x, (map_pc_buffer_.end()-1)->points[i].y, (map_pc_buffer_.end()-1)->points[i].z);
        }
                }
        );
        // for (size_t i = 0; i < (map_pc_buffer_.end()-1)->size(); i++){
        //     points_tf[i]=Eigen::Vector3d((map_pc_buffer_.end()-1)->points[i].x, (map_pc_buffer_.end()-1)->points[i].y, (map_pc_buffer_.end()-1)->points[i].z);
        // }
        map_pc_eigen_buffer_.push_back(points_tf);
    }
    //add functionality for pitch and yaw angle

    //angle_roll_ += droll;
    // angle_pitch_ += dpitch;
    // angle_yaw_ += dyaw;
    ROS_INFO_STREAM("Angle: " << angle_roll_/M_PI*180);

    // Detect completed full turns and keep remainder in angle_
    static int full_turns_roll = 0;
    static int full_turns_pitch = 0;
    static int full_turns_yaw = 0;
    static bool init_rotated = false;
    if (std::abs(angle_roll_) >= 2.0 * M_PI) {
        const int dir = (angle_roll_ > 0.0) ? 1 : -1;           // +CCW, -CW
        angle_roll_ -= dir * 2.0 * M_PI;                        // keep remainder for continued integration
        ++full_turns_roll;
        init_rotated = true;
        ROS_INFO_STREAM("Full self-rotation (roll) detected (" << (dir>0?"+":"-") << "1), total=" << full_turns_roll);
    }
    if (std::abs(angle_pitch_) >= 2.0 * M_PI){
        const int dir = (angle_pitch_ > 0.0) ? 1 : -1;           // +CCW, -CW
        angle_pitch_ -= dir * 2.0 * M_PI;                        // keep remainder for continued integration
        ++full_turns_pitch;
        init_rotated = true;
        ROS_INFO_STREAM("Full self-rotation (pitch) detected (" << (dir>0?"+":"-") << "1), total=" << full_turns_pitch);
   
    }
    if (std::abs(angle_yaw_) >= 2.0 * M_PI){
        const int dir = (angle_yaw_ > 0.0) ? 1 : -1;           // +CCW, -CW
        angle_yaw_ -= dir * 2.0 * M_PI;                        // keep remainder for continued integration
        ++full_turns_yaw;
        init_rotated = true;
        ROS_INFO_STREAM("Full self-rotation (yaw) detected (" << (dir>0?"+":"-") << "1), total=" << full_turns_yaw);
   
    }
    if (init_rotated){
        int size = map_pose_buffer_.size();
        int k = std::max(1, size/12);
        int i = 0;
        auto pose_it = map_pose_buffer_.begin();
        auto pc_it = map_pc_buffer_.begin();
        std::vector<std::vector<Eigen::Vector3d>> eigen_buffer_reduced;
        std::vector<pcl::PointCloud<PointType>> pc_buffer_reduced;
        std::vector<Sophus::SE3d> pose_buffer_reduced;
        for (auto it = map_pc_eigen_buffer_.begin(); it != map_pc_eigen_buffer_.end(); it++, pose_it++, pc_it++){
            //pcl::transformPointCloud(*it, *it, pose_it->translation(), pose_it->unit_quaternion());
            if (i==k) {i=0;}
            else {i++; continue;}
            ROS_INFO_STREAM("call");
            eigen_buffer_reduced.push_back(*it);
            pc_buffer_reduced.push_back(*pc_it);
            pose_buffer_reduced.push_back(*pose_it);

        }
        ROS_INFO_STREAM("initial: " << eigen_buffer_reduced.size());
        //performGraphSLAM(eigen_buffer_reduced, pose_buffer_reduced);
        
        for (size_t i = 0; i < pc_buffer_reduced.size(); i++){
            pcl::transformPointCloud(pc_buffer_reduced[i], pc_buffer_reduced[i], pose_buffer_reduced[i].translation(), pose_buffer_reduced[i].unit_quaternion());
            if (ikdtree_ptr_->Root_Node == nullptr){
                ikdtree_ptr_->Build(pc_buffer_reduced[i].points);
            }
            else{
                int num_points = ikdtree_ptr_->Add_Points(pc_buffer_reduced[i].points, true);
                ROS_INFO_STREAM(num_points << " points added to the ikd-Tree.");
            }
        }
        auto map = boost::make_shared<pcl::PointCloud<PointType>>();
        ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, map->points, NOT_RECORD);
        sensor_msgs::PointCloud2 map_out;
        pcl::toROSMsg(*map, map_out);
        map_out.header.frame_id = odom_frame_;
        map_out.header.stamp = stamp;
        map_pub_.publish(map_out);  
        map_initialized_ = true;
    }
}
//ROS_INFO_STREAM("Angle: " << angle_/M_PI*180);
/*---------------------------
|   PUBLISHING FUNCTIONS    |
---------------------------*/

void LIONode::publishPointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const ros::Time &stamp, Sophus::SE3d &pose){
    
    //current PointCloud
    auto pc_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc_tf->points.reserve(pc->points.size());
    pc_tf->header = pc->header;
    pc_tf->header.frame_id = odom_frame_;
    pc_tf->is_dense = pc->is_dense;
    pcl::transformPointCloud(*pc, *pc_tf, pose.translation(), pose.unit_quaternion(), false);
    sensor_msgs::PointCloud2 scan_out;
    pcl::toROSMsg(*pc_tf, scan_out);
    scan_out.header.frame_id = odom_frame_;
    scan_out.header.stamp = stamp;
    pc_pub_.publish(scan_out);
    // pc_pub_.publish(scan_out);
    // sensor_msgs::PointCloud2 pc_msg;
    // pcl::toROSMsg(*pc, pc_msg);
    // pc_msg.header.frame_id = odom_frame_;
    // pc_msg.header.stamp = stamp;
    // const auto base2odom = LookupTransform(prev_odom_frame_, base_frame_, stamp);
    // pcl::transformPointCloud(*pc, *pc, base2odom.translation(), base2odom.unit_quaternion());
    // pc->header.frame_id = odom_frame_;
    // sensor_msgs::PointCloud2 scan_out;
    // pcl::toROSMsg(*pc, scan_out);
    // scan_out.header.frame_id = odom_frame_;
    // scan_out.header.stamp = stamp;
    // pc_pub_.publish(scan_out);

    //current map
    auto map = boost::make_shared<pcl::PointCloud<PointType>>();
    ikdtree_ptr_->flatten(ikdtree_ptr_->Root_Node, map->points, NOT_RECORD);
    sensor_msgs::PointCloud2 map_out;
    pcl::toROSMsg(*map, map_out);
    map_out.header.frame_id = odom_frame_;
    map_out.header.stamp = stamp;
    map_pub_.publish(map_out);   
}
void LIONode::publishOdometry(const ros::Time &stamp){
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = base_frame_;
    transform_msg.child_frame_id = odom_frame_;
    auto pose = poses_.back().inverse();//LookupTransform(base_frame_, prev_odom_frame_, stamp);
    transform_msg.transform = sophusToTransform(pose);
    tf2_broadcaster_.sendTransform(transform_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh;
    LIONode node(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}




//DEBUGGING
void LIONode::preprocess(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out, double start_time){
    //imu2lidar_tf = lookupTransform("pandar_frame", "imu_frame");
    //calculate LiDAR average rotation velocity
    //tf2::Vector3 om = imu2lidar_tf * tf2::Vector3(imu_x_/imu_num_, imu_y_/imu_num_, imu_z_/imu_num_);
    //tf2::Vector3 om = tf2::quatRotate(imu2lidar_rot, tf2::Vector3(imu_x_/imu_num_, imu_y_/imu_num_, imu_z_/imu_num_));
    tf2::Vector3 om = tf2::Vector3(imu_x_/imu_num_, imu_y_/imu_num_, imu_z_/imu_num_);
    imu_x_=0;
    imu_y_=0;
    imu_z_=0;
    imu_num_=0;

    for (size_t i = 0; i < pc_in->points.size(); ++i) {
        double point_x=pc_in->points[i].x;
        double point_y=pc_in->points[i].y;
        double point_z=pc_in->points[i].z;
        //std::cout << pc_in->points[i].timestamp-start_time <<std::endl;
        //std::cout << "om: " << om.x() << om.y() << om.z() << std::endl;
        tf2::Vector3 rot = om*(pc_in->points[i].timestamp-start_time); //in local frame
        //std::cout << rot.x << " " << rot.y() << " " << rot.z() << std::endl;
        pcl::PointXYZ point_corr;
        point_corr.x = (cos(rot.y())*cos(rot.z())) * point_x + (sin(rot.x())*sin(rot.y())*cos(rot.z())-cos(rot.x())*sin(rot.z())) * point_y + (cos(rot.x())*sin(rot.y())*cos(rot.z())+sin(rot.x())*sin(rot.z()))*point_z;
        point_corr.y = (cos(rot.y())*sin(rot.z())) * point_x + (sin(rot.x())*sin(rot.y())*sin(rot.z())+cos(rot.x())*cos(rot.z())) * point_y + (cos(rot.x())*sin(rot.y())*sin(rot.z())-sin(rot.x())*cos(rot.z()))*point_z;
        point_corr.z = (-sin(rot.y())) * point_x + (sin(rot.x())*cos(rot.y())) * point_y + (cos(rot.x())*cos(rot.y())) * point_z;


        pc_out->push_back(point_corr);
    }
}
