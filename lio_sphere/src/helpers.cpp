#include "lio_node.hpp"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

Sophus::SE3d LIONode::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame, const ros::Time &time) const {
    std::string err_msg;
    if (tf2_buffer_._frameExists(source_frame) &&  //
        tf2_buffer_._frameExists(target_frame) &&  //
        tf2_buffer_.canTransform(target_frame, source_frame, time, &err_msg)) {
        try {
            auto tf = tf2_buffer_.lookupTransform(target_frame, source_frame, time);
            return transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    ROS_WARN("Failed to find tf between %s and %s. Reason=%s", target_frame.c_str(),
             source_frame.c_str(), err_msg.c_str());
    return {};
}
void LIONode::voxelize(const pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out, float voxel_size){
    pcl::ApproximateVoxelGrid<PointType> sor;
    sor.setInputCloud(pc_in);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*pc_out);
}
void LIONode::dist_filter(pcl::PointCloud<PointType>::Ptr& pc_in, pcl::PointCloud<PointType>::Ptr& pc_out,
                                        double max_range,
                                        double min_range) {
    for (const auto& pt : pc_in->points) {
        double norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (norm < max_range && norm > min_range) {
            pc_out->points.push_back(pt);
        }
    }
}
float DET_RANGE = 100.0f;
const float MOV_THRESHOLD = 1.5f;
std::vector<BoxPointType> cub_needrm;
Eigen::Vector3d pos_LiD;
double cube_len = 40;
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void LIONode::move_map(int &kdtree_delete_counter, const Sophus::SE3d &current_pose)
{
    pos_LiD = current_pose.translation();
    cub_needrm.clear();
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    //std::cout << "deletion running";
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree_ptr_->Delete_Point_Boxes(cub_needrm);
}
std::tuple<Sophus::SE3d, Sophus::SE3d> LIONode::getInitialGuess(const ros::Time &timestamp){
    const auto init_pose = LookupTransform(base_frame_, prev_odom_frame_, timestamp).inverse();
    // geometry_msgs::TransformStamped transform_msg;
    // transform_msg.header.stamp = timestamp;
    // transform_msg.header.frame_id = odom_frame_;
    // transform_msg.child_frame_id = "initial_guess";
    // //auto pose = LookupTransform(base_frame_, prev_odom_frame_, stamp).inverse();
    // transform_msg.transform = sophusToTransform(init_pose);
    // tf2_broadcaster_.sendTransform(transform_msg);

    if (!pose_initialized_){
        prev_init_pose_ = init_pose;
        prev_pose_ = init_pose;
        pose_initialized_ = true;
        first_pose_=init_pose;
        //angle_=0.0;
    }
    const auto prediction = prev_init_pose_.inverse() * init_pose;
    double ang_change=2*acos(prediction.unit_quaternion().w());
    //if (ang_change > 0.2) angle_ += 2*acos(prediction.unit_quaternion().w());
    Sophus::SE3d prev_pose;
    if (no_match_counter_ < 1) {
        prev_pose = !poses_.empty() ? poses_.back() : prev_init_pose_;
        
    }
    else{
        prev_pose = prev_pose_;}
    const auto new_pose = prev_pose * prediction; 
    prev_pose_ = new_pose;
    prev_init_pose_ = init_pose;
    return {new_pose, prediction};
}
double LIONode::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    if (no_match_counter_ < 1) return adaptive_threshold_.ComputeThreshold();
    else return adaptive_threshold_.ComputeThresholdNoMatch(no_match_counter_);
}
bool LIONode::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}
bool LIONode::check_if_registration(){
    if (init_counter_<40){
        init_counter_++;
        return true;
    }
    if (counter_ == 0){
        counter_++;
        return true;
    }
    else{
        if(counter_==num_iterations_between_) counter_=0;
        counter_++;
        return false;
    }
}