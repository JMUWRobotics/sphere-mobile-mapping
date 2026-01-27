#include "lio_node.hpp"
#include <algorithm>
#include <cmath>

using Voxel = Eigen::Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};





void compare_transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &frame_pcl, std::vector<Eigen::Vector3d> &frame_eigen, const Sophus::SE3d &transform){
    const double voxel_size = 0.5;

    
    EigenTransformation(frame_eigen, transform);
        
        
    PCLTransformation(frame_pcl, transform);


}
    
void EigenTransformation(std::vector<Eigen::Vector3d> &frame, const Sophus::SE3d &transform) {
    std::transform(frame.cbegin(), frame.cend(), frame.begin(),
                   [&](const auto &point) { return transform * point; });
    ROS_INFO_STREAM("Eigen Transformation: " << frame.begin()->x() << ", " << frame.begin()->y() << ", " << frame.begin()->z());
}

void PCLTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &frame, const Sophus::SE3d &transform){
    pcl::PointCloud<pcl::PointXYZ> frame_tf;
    frame_tf.reserve(frame->size());
    pcl::transformPointCloud(*frame, frame_tf, transform.translation(), transform.unit_quaternion());
    ROS_INFO_STREAM("PCL Transformation: " << frame_tf.points[0].x << ", " << frame_tf.points[0].y << ", " << frame_tf.points[0].z);
}