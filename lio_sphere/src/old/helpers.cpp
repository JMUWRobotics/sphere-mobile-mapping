#include "lio_node.hpp"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

Sophus::SE3d LIONode::LookupTransform(const std::string &target_frame,
                                      const std::string &source_frame, 
                                      const ros::Time &time) const 
{
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

void LIONode::voxelize(const pcl::PointCloud<PointType>::Ptr& pc_in, 
                       pcl::PointCloud<PointType>::Ptr& pc_out, 
                       float voxel_size)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(pc_in);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*pc_out);
}

void LIONode::dist_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out,
                          double max_range,
                          double min_range) 
{
    for (const auto& pt : pc_in->points) {
        double norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (norm < max_range && norm > min_range) {
            pc_out->points.push_back(pt);
        }
    }
}

// Overload for custom point type (if native deskewing is enabled)
void LIONode::dist_filter(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, 
                          pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_out,
                          double max_range,
                          double min_range) 
{
    for (const auto& pt : pc_in->points) {
        double norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (norm < max_range && norm > min_range) {
            pc_out->points.push_back(pt);
        }
    }
}

void LIONode::move_map(int &kdtree_delete_counter, 
                       const Sophus::SE3d &current_pose)
{
    float DET_RANGE = config_.max_range;
    float MOV_THRESHOLD = config_.sliding_map_gamma;
    double cube_len = config_.sliding_map_L;
    pos_lid_ = current_pose.translation();
    cub_needrm_.clear();
    if (!localmap_initialized_) {
        for (int i = 0; i < 3; i++) {
            local_map_points_.vertex_min[i] = pos_lid_(i) - cube_len / 2.0;
            local_map_points_.vertex_max[i] = pos_lid_(i) + cube_len / 2.0;
        }
        localmap_initialized_ = true;
        return;
    }
    // Check if we need to move 
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_lid_(i) - local_map_points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lid_(i) - local_map_points_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
    }
    // Early stop 
    if (!need_move) {
        return;
    }
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = local_map_points_;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = local_map_points_;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        }
    }
    local_map_points_ = New_LocalMap_Points;
    ROS_WARN_STREAM("Distance exceeded, deleting far away pts from global map");
    if(cub_needrm_.size() > 0) {
        kdtree_delete_counter = ikdtree_ptr_->Delete_Point_Boxes(cub_needrm_);
    } 
}

std::tuple<Sophus::SE3d, Sophus::SE3d> LIONode::getInitialGuess(const ros::Time &timestamp)
{
    Sophus::SE3d init_pose;
    if (config_.initial_method==KALMAN) {
        init_pose = LookupTransform(config_.base_frame, config_.kalman_odom_frame, timestamp).inverse() * tf_lidar2base_;
    } else {
        init_pose = LookupTransform(config_.base_frame, config_.imu_odom_frame, timestamp).inverse() * tf_lidar2base_;
    }

    Sophus::SE3d prediction;
    {
        std::lock_guard<std::mutex> lock(mtx_pose_);
        if (!pose_initialized_){
            prev_init_pose_ = init_pose;
            prev_pose_ = init_pose;
            pose_initialized_ = true;
        }
        prediction = prev_init_pose_.inverse() * init_pose;
    }
    Sophus::SE3d prev_pose;
    {
        std::lock_guard<std::mutex> lock(mtx_pose_);
        prev_pose = !poses_.empty() ? poses_.back() : prev_init_pose_;
    }
    auto new_pose = prev_pose * prediction;
    {
        std::lock_guard<std::mutex> lock(mtx_pose_);
        prev_pose_ = new_pose;
        prev_init_pose_ = init_pose;
    }
    return {new_pose, prediction};
}
