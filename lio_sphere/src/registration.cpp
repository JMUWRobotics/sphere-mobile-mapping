#include <geometry_msgs/Pose.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include "lio_node.hpp"

bool LIONode::registerPoints(const pcl::PointCloud<PointType>::Ptr &pc_in, const Sophus::SE3d &initial_guess, Sophus::SE3d &registered_pose, double max_correspondence_distance, int max_num_iterations, double sigma){
    
    double estimation_threshold = 0.0001;
    if (ikdtree_ptr_->Root_Node == nullptr) {registered_pose = initial_guess; return true;}
    using clock = std::chrono::high_resolution_clock;
    auto t0 = clock::now();

    
    pcl::PointCloud<PointType> source;
    source.points.reserve(pc_in->points.size());
    pcl::transformPointCloud(*pc_in, source, initial_guess.translation(), initial_guess.unit_quaternion(), false);
    
    PointVector nearest_neighbour;
    std::vector<float> distance;
    std::vector<Eigen::Vector3d> src;
    std::vector<Eigen::Vector3d> tgt;
    std::vector<float> dis;

    nearest_neighbour.reserve(1);
    distance.reserve(1);
    src.reserve(source.points.size());
    tgt.reserve(source.points.size());
    dis.reserve(source.points.size());

    const size_t N = source.points.size();
    std::vector<Eigen::Vector3d> src_buf(N), tgt_buf(N);
    std::vector<float> dis_buf(N);
    std::vector<char> valid(N, 0);

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    int num_it;
    long long sum_nn = 0;
    int size = 0;
    float eval_max_dist = 0.0;
    for (int j = 0; j < max_num_iterations; ++j) {
        eval_max_dist = 0.0;
        auto t2 = clock::now();
        src.clear();
        tgt.clear();
        // dis.clear();
        src_buf.clear();
        tgt_buf.clear();
        // dis_buf.clear();
        valid.clear();
        tbb::parallel_for(tbb::blocked_range<size_t>(0, N),
            [&](const tbb::blocked_range<size_t> &range){
                // thread-local temporaries to avoid repeated allocations and contention
                thread_local PointVector nearest_neighbour;
                thread_local std::vector<float> distance;
                for (size_t i = range.begin(); i < range.end(); ++i) {
                nearest_neighbour.clear();
                // distance.clear();
                ikdtree_ptr_->Nearest_Search(source.points[i], 1, nearest_neighbour, distance, sigma);
                if (!nearest_neighbour.empty()) {
                    // store into per-index buffers; mark valid
                    src_buf[i] = Eigen::Vector3d(source.points[i].x, source.points[i].y, source.points[i].z);
                    const auto &nn = nearest_neighbour[0];
                    tgt_buf[i] = Eigen::Vector3d(nn.x, nn.y, nn.z);
                    valid[i] = 1;
                    // dis_buf[i] = distance[0];
                }
                }
            });
            std::vector<Eigen::Vector3d> src, tgt;
            src.reserve(N);
            tgt.reserve(N);
            for (size_t i = 0; i < N; ++i) {
            if (valid[i]) {
                src.push_back(src_buf[i]);
                tgt.push_back(tgt_buf[i]);
                // dis.push_back(dis_buf[i]);
            }
            }
        auto t3 = clock::now();
        size += src.size();
        // auto max_distance = std::max_element(dis.begin(), dis.end());
        // if (max_distance == dis.end()) {ROS_INFO_STREAM("distances empty");}
        // else {ROS_INFO_STREAM("dis ikdtree: " << *max_distance);}
        // Equation (11)
        sum_nn += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
        //ROS_INFO_STREAM("src: " << src.size() << ", tgt: " << tgt.size());
        const auto &[JTJ, JTr] = BuildLinearSystem(src, tgt, sigma/3);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        //std::cout << src.size();
        // Equation (12)
        //TransformPoints(estimation, source);
        pcl::transformPointCloud(source, source, estimation.translation(), estimation.unit_quaternion());
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (dx.norm() < estimation_threshold) break;
        if (j == max_num_iterations-1){ ROS_WARN("registration failed"); 
            //T_icp=initial_guess;
            registered_pose = initial_guess;
            return false;
            }
        num_it = j;
    }
    // Spit the final transformation
    //ROS_INFO_STREAM("number of iterations: " << num_it);
    auto t1=clock::now();
    //ROS_INFO_STREAM("Runtime NN-Search ikd Tree: " << static_cast<double>(sum_nn) << " us, Scan size: " << size << "Map size: " << ikdtree_ptr_->size() << ", validnum: " << ikdtree_ptr_->validnum());
    //ROS_INFO_STREAM("distance: " << max_correspondence_distance);
    //ROS_INFO_STREAM("Runtime Registration ikd Tree: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count()) << " us");
    registered_pose = T_icp * initial_guess;
    return true;
}

std::tuple<Eigen::Matrix6d, Eigen::Vector6d> LIONode::BuildLinearSystem(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    double kernel) {
    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]);
        return std::make_tuple(J_r, residual);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, source.size()},
        // Identity
        ResultTuple(),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual2) {
                return square(kernel) / square(kernel + residual2);
            };
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, residual] = compute_jacobian_and_residual(i);
                const double w = Weight(residual.squaredNorm());
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * residual;
            }
            return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });

    return std::make_tuple(JTJ, JTr);
}