#include <geometry_msgs/Pose.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "lio_node.hpp"
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
bool LIONode::registerPointsNanoGICP(const pcl::PointCloud<PointType>::Ptr &pc_in, const Sophus::SE3d &initial_guess, Sophus::SE3d &registered_pose, double max_correspondence_distance, int max_num_iterations, double sigma){
    
    double estimation_threshold = 0.0005;
    if (ikdtree_ptr_->Root_Node == nullptr) {registered_pose = initial_guess; return true;}
    using clock = std::chrono::high_resolution_clock;
    auto t0 = clock::now();

    
    const size_t N = pc_in->points.size();
    const int kNN = 5;
    std::vector<Eigen::Vector3d> src_buf(N);
    std::vector<Eigen::Vector3d> tgt_buf(kNN*N);
    std::vector<bool> valid(N);
    std::vector<int> nn_count(N,0);
    auto source_pc = boost::make_shared<pcl::PointCloud<PointType>>();
    auto target_pc = boost::make_shared<pcl::PointCloud<PointType>>();
    source_pc->points.reserve(pc_in->points.size());
    target_pc->points.reserve(kNN*pc_in->points.size());



    tbb::parallel_for(tbb::blocked_range<size_t>(0, N),
            [&](const tbb::blocked_range<size_t> &range){
                // thread-local temporaries to avoid repeated allocations and contention
                thread_local PointVector nearest_neighbour(kNN);
                thread_local std::vector<float> distance;
                for (size_t i = range.begin(); i < range.end(); ++i) {
                nearest_neighbour.clear();
                distance.clear();
                ikdtree_ptr_->Nearest_Search(pc_in->points[i], kNN, nearest_neighbour, distance, sigma);
                if (!nearest_neighbour.empty()) {
                    const int num = std::min<int>(kNN, static_cast<int>(nearest_neighbour.size()));
                    for (int k = 0; k<num; ++k){
                        const auto &nn = nearest_neighbour[k];
                        tgt_buf[k+i*kNN] = Eigen::Vector3d(nn.x, nn.y, nn.z);
                    }
                    src_buf[i] = initial_guess * Eigen::Vector3d(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
                    
                    nn_count[i]=num;
                    valid[i] = 1;
                    // dis_buf[i] = distance[0];
                }
                }
            });
            std::vector<Eigen::Vector3d> source, target;
            source.reserve(N);
            target.reserve(kNN*N);
            for (size_t i = 0; i < N; ++i) {
            if (valid[i]) {
                source.push_back(src_buf[i]);
                for(int k=0; k<nn_count[i]; k++){
                target.push_back(tgt_buf[i*kNN+k]);
                source_pc->points.push_back(pcl::PointXYZ(src_buf[i].x(), src_buf[i].y(), src_buf[i].z()));
                target_pc->points.push_back(pcl::PointXYZ(tgt_buf[i*kNN+k].x(), tgt_buf[i*kNN+k].y(), tgt_buf[i*kNN+k].z()));
                }
                // dis.push_back(dis_buf[i]);
            }
            }
    //debug the generated target point cloud
    if (!source_pc->points.empty()) {
        auto source_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        source_cloud->points.reserve(source_pc->points.size());
        for (const auto &v : source_pc->points) {
            source_cloud->points.emplace_back(pcl::PointXYZ(v.x, v.y, v.z));
        }
        source_cloud->is_dense = true;
        sensor_msgs::PointCloud2 source_msg;
        pcl::toROSMsg(*source_pc, source_msg);
        source_msg.header.stamp = ros::Time::now();
        source_msg.header.frame_id = odom_frame_; // already transformed by initial_guess
        source_pc_pub_.publish(source_msg);
    }
    if (!target_pc->points.empty()) {
        auto target_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        target_cloud->points.reserve(target_pc->points.size());
        for (const auto &v : target_pc->points) {
            target_cloud->points.emplace_back(pcl::PointXYZ(v.x, v.y, v.z));
        }
        target_cloud->is_dense = true;
        sensor_msgs::PointCloud2 target_msg;
        pcl::toROSMsg(*target_pc, target_msg);
        target_msg.header.stamp = ros::Time::now();
        target_msg.header.frame_id = odom_frame_; // map frame
        target_pc_pub_.publish(target_msg);
    }
    nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    Sophus::SE3d T_icp = Sophus::SE3d();
    m_nano_gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
    //m_nano_gicp.setNumThreads(m_nano_gicp.num_threads_);
    m_nano_gicp.setCorrespondenceRandomness(20);
    m_nano_gicp.setMaximumIterations(max_num_iterations);
    m_nano_gicp.setTransformationEpsilon(estimation_threshold);
    ROS_INFO_STREAM("Num Threads: " << m_nano_gicp.num_threads_);

    pcl::PointCloud<PointType> dummy;
    m_nano_gicp.setInputSource(source_pc);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(target_pc);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(dummy);
    double score = m_nano_gicp.getFitnessScore();
    ROS_INFO_STREAM("Fitness Score: " << score);
    if(m_nano_gicp.hasConverged())// && score < m_nano_gicp.icp_score_threshold)
    {
    Eigen::Matrix4f pose_betweenf = m_nano_gicp.getFinalTransformation(); //float
    Eigen::Matrix4d pose_betweend = m_nano_gicp.getFinalTransformation().cast<double>(); //double
    registered_pose=Sophus::SE3d::fitToSE3(pose_betweend)*initial_guess;
    return true;
    }
    else{
        registered_pose = initial_guess;
        return false;
    }

    // tbb::parallel_for(tbb::blocked_range<size_t>(0, pc_in->points.size()),
    // [&](const tbb::blocked_range<size_t> &range){
    //     for (size_t i = range.begin(); i < range.end(); ++i) {
    //         const auto &p = pc_in->points[i];
    //         source[i] = initial_guess * Eigen::Vector3d(p.x, p.y, p.z);
    //     }
    // });
    //for simplicity: at first only use the nearest neighbours of the source points in the map as target vector
    //target kd tree
    //EigenPointCloudAdaptor target_adaptor{target};  // adaptor holds a reference; keep 'target' alive
    // KDTree_t target_kdtree_(
    //     3,                                  // dimensionality
    //     target_adaptor,
    //     nanoflann::KDTreeSingleIndexAdaptorParams(10));
    // target_kdtree.buildIndex();
    // kd_tree_t target_kdtree(3, target, 10);

    // // ICP-loop
    // Sophus::SE3d T_icp = Sophus::SE3d();
    // int num_it;
    // long long sum_nn = 0;
    // int size = 0;
    // float eval_max_dist = 0.0;
    // // ---- TBB correspondence search (similar to Open3D GetRegistrationResultAndCorrespondences) ----
    // std::vector<Eigen::Vector2i> correspondences;// (source_index, target_index)
    // correspondences.reserve(source.size());
    
    // for (int j = 0; j < max_num_iterations; ++j) {
    //     double error2 = 0.0; // sum of squared distances
    //     std::vector<int> match_index(source.size(), -1);
    //     std::vector<double> match_dist2(source.size(), 0.0);

    //     if (max_correspondence_distance > 0.0 && !source.empty() && !target.empty()) {
    //         const double max_dist_sq = max_correspondence_distance * max_correspondence_distance;
    //         tbb::parallel_for(tbb::blocked_range<size_t>(0, source.size()),
    //             [&](const tbb::blocked_range<size_t> &range){
    //                 std::vector<size_t> nn_index(1);
    //                 std::vector<double> nn_dist2(1);
    //                 for (size_t i = range.begin(); i < range.end(); ++i){
    //                     const double query[3] = {source[i].x(), source[i].y(), source[i].z()};
    //                     size_t found = target_kdtree.index->knnSearch(query, 1, nn_index.data(), nn_dist2.data());
    //                     if (found > 0 && nn_dist2[0] <= max_dist_sq){
    //                         match_index[i] = static_cast<int>(nn_index[0]);
    //                         match_dist2[i] = nn_dist2[0];
    //                     }
    //                 }
    //             });
    //         // Build correspondences vector and accumulate error
    //         correspondences.clear();
    //         for (size_t i = 0; i < source.size(); ++i){
    //             if (match_index[i] >= 0){
    //                 correspondences.emplace_back(static_cast<int>(i), match_index[i]);
    //                 error2 += match_dist2[i];
    //             }
    //         }
    //     }

    //     double fitness = 0.0;
    //     double inlier_rmse = 0.0;
    //     if (!correspondences.empty()){
    //         fitness = static_cast<double>(correspondences.size()) / static_cast<double>(source.size());
    //         inlier_rmse = std::sqrt(error2 / static_cast<double>(correspondences.size()));
    //     }
    //     // DEBUG (optional): ROS_INFO_STREAM("Correspondences: " << correspondences.size() << ", fitness=" << fitness << ", rmse=" << inlier_rmse);
    //     // Build linear system (GICP style) including robust weights
    //     const auto &[JTJ, JTr, residual_sum] = BuildLinearSystemGICP(source, target, covariances_source, covariances_target, correspondences, sigma/3.0);
    //     const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
    //     const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
    //     //std::cout << src.size();
    //     // Equation (12)
    //     transformPoints(estimation, source);
    //     transformCovariances(estimation, covariances_source);
    //     //transform covariances accordingly Open3d implementation
    //     T_icp = estimation * T_icp;
    //     // Termination criteria
    //     if (dx.norm() < estimation_threshold) break;
    //     if (j == max_num_iterations-1){ ROS_WARN("registration failed"); 
    //         //T_icp=initial_guess;
    //         registered_pose = initial_guess;
    //         return false;
    //         }
    //     num_it = j;
    // }
    // // Spit the final transformation
    // //ROS_INFO_STREAM("number of iterations: " << num_it);
    // auto t1=clock::now();
    // //ROS_INFO_STREAM("Runtime NN-Search ikd Tree: " << static_cast<double>(sum_nn) << " us, Scan size: " << size << "Map size: " << ikdtree_ptr_->size() << ", validnum: " << ikdtree_ptr_->validnum());
    // //ROS_INFO_STREAM("distance: " << max_correspondence_distance);
    // //ROS_INFO_STREAM("Runtime Registration ikd Tree: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count()) << " us");
    // registered_pose = T_icp * initial_guess;
    // return true;
}