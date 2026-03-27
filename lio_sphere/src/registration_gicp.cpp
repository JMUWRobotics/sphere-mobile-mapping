#include <geometry_msgs/Pose.h>
#include <cmath>
#include <limits>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include "lio_node.hpp"

namespace
{
    /** @brief Computes the inverse square root of a 3x3 symmetric positive-definite matrix
     *         via Cholesky decomposition, with progressive regularisation fallback.
     * @param M Input symmetric positive-definite 3x3 matrix.
     * @return M^{-1/2} (identity matrix if decomposition fails after all retries).
     */
    inline Eigen::Matrix3d computeInverseSqrtSPD(const Eigen::Matrix3d &M)
    {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d M_reg = M;

        for (int attempt = 0; attempt < 6; ++attempt)
        {
            Eigen::LLT<Eigen::Matrix3d> llt(M_reg);
            if (llt.info() == Eigen::Success)
            {
                return llt.matrixL().solve(I);
            }
            const double eps = std::pow(10.0, -9 + attempt);
            M_reg = M + eps * I;
        }

        return I;
    }
}

void LIONode::transformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points)
{
    // Apply SE3 transform to each point in parallel using TBB.
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                points[i] = T * points[i];
            }
        });
}

void LIONode::transformCovariances(const Sophus::SE3d &T, std::vector<Eigen::Matrix3d> &covariances)
{
    // Apply SE3 transform to each matrix in parallel using TBB.
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, covariances.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                covariances[i] = T.rotationMatrix() * covariances[i] * T.rotationMatrix().transpose();
            }
        });
}

inline Eigen::Matrix3d LIONode::GetRotationFromE1ToX(const Eigen::Vector3d &x)
{
    const Eigen::Vector3d e1{1, 0, 0};
    const Eigen::Vector3d v = e1.cross(x);
    const double c = e1.dot(x);
    if (c < -0.99)
    {
        // Then means that x and e1 are in the same direction
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d sv;
    sv << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    const double factor = 1 / (1 + c);
    return Eigen::Matrix3d::Identity() + sv + (sv * sv) * factor;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> LIONode::construct_source_and_target_pc(const pcl::PointCloud<PointType>::Ptr &pc_in)
{
    const size_t N = pc_in->points.size();
    const int kNN = config_.target_NN_k;
    std::vector<bool> valid(N);
    std::vector<std::vector<Eigen::Vector3d>> target_all(N);

    std::vector<Eigen::Vector3d> source(N), target;
    target.reserve(kNN * N);
    // calculate covariances for target points
    // get target points as box range from the ikd tree and voxelize them
    // build target kd tree
    tbb::parallel_for(tbb::blocked_range<size_t>(0, N),
        [&](const tbb::blocked_range<size_t> &range)
        {
            // thread-local temporaries to avoid repeated allocations and contention
            thread_local PointVector nearest_neighbour(kNN);
            thread_local std::vector<float> distance;
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                nearest_neighbour.clear();
                distance.clear();
                ikdtree_ptr_->Nearest_Search(pc_in->points[i], kNN, nearest_neighbour, distance, 5);
                if (!nearest_neighbour.empty())
                {
                    const int num = std::min<int>(kNN, static_cast<int>(nearest_neighbour.size()));
                    target_all[i].reserve(num);
                    for (int k = 0; k < num; ++k)
                    {
                        const auto &nn = nearest_neighbour[k];
                        target_all[i].push_back(Eigen::Vector3d(nn.x, nn.y, nn.z));
                    }
                    valid[i] = true;
                }
                source[i] = Eigen::Vector3d(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            }
        }
    );
    for (size_t i = 0; i < N; ++i)
    {
        if (valid[i])
        {
            target.insert(target.end(), target_all[i].begin(), target_all[i].end());
        }
    }
    return std::make_pair(source, target);
}

void LIONode::find_correspondences_and_calculate_error(std::vector<Eigen::Vector3d> &source, 
                                                       std::vector<Eigen::Vector2i> &correspondences, 
                                                       const kd_tree_t &target_kdtree, 
                                                       double &max_correspondence_distance, 
                                                       double &error2)
{
    std::vector<int> match_index(source.size(), -1);
    std::vector<double> match_dist2(source.size(), 0.0);

    const double max_dist_sq = max_correspondence_distance * max_correspondence_distance;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, source.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            std::vector<size_t> nn_index(1);
            std::vector<double> nn_dist2(1);
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                const double query[3] = {source[i].x(), source[i].y(), source[i].z()};
                size_t found = target_kdtree.index->knnSearch(query, 1, nn_index.data(), nn_dist2.data());
                if (found > 0 && nn_dist2[0] <= max_dist_sq)
                {
                    match_index[i] = static_cast<int>(nn_index[0]);
                    match_dist2[i] = nn_dist2[0];
                }
            }
        }
    );
    // Build correspondences vector and accumulate error
    correspondences.clear();
    for (size_t i = 0; i < source.size(); ++i)
    {
        if (match_index[i] >= 0)
        {
            correspondences.emplace_back(static_cast<int>(i), match_index[i]);
            error2 += match_dist2[i];
        }
    }
}

double LIONode::ComputeRMSE(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    const std::vector<Eigen::Matrix3d> &covariances_source,
    const std::vector<Eigen::Matrix3d> &covariances_target,
    const std::vector<Eigen::Vector2i> &corres) const
{
    if (corres.empty())
    {
        return 0.0;
    }
    double err = 0.0;
    for (const auto &c : corres)
    {
        const Eigen::Vector3d &vs = source[c[0]];
        const Eigen::Matrix3d &Cs = covariances_source[c[0]];
        const Eigen::Vector3d &vt = target[c[1]];
        const Eigen::Matrix3d &Ct = covariances_target[c[1]];
        const Eigen::Vector3d d = vs - vt;
        const Eigen::Matrix3d M = Ct + Cs;
        const Eigen::Matrix3d W = computeInverseSqrtSPD(M);
        err += d.transpose() * W * d;
    }
    return std::sqrt(err / (double)corres.size());
}

bool LIONode::registerPointsGICP(const pcl::PointCloud<PointType>::Ptr &pc_in, 
                                 const Sophus::SE3d &initial_guess, 
                                 Sophus::SE3d &registered_pose, 
                                 double kernel, 
                                 double max_correspondence_distance, 
                                 int max_num_iterations, 
                                 double &matching_error)
{
    auto gicp_t0 = std::chrono::high_resolution_clock::now();
    {
        std::lock_guard<std::mutex> lock(mtx_kdtree_);
        if (ikdtree_ptr_->Root_Node == nullptr)
        { 
            registered_pose = initial_guess;
            return true;
        }
    }

    auto pc_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc_tf->points.reserve(pc_in->points.size());
    pcl::transformPointCloud(*pc_in, *pc_tf, initial_guess.translation(), initial_guess.unit_quaternion());
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
    {
        std::lock_guard<std::mutex> lock(mtx_kdtree_);
        std::tie(source, target) = construct_source_and_target_pc(pc_tf);
    }

    if (source.empty() || target.empty())
    {
        ROS_WARN_STREAM("GICP skipped: empty source/target set (source=" << source.size() << ", target=" << target.size() << ")");
        registered_pose = initial_guess;
        matching_error = std::numeric_limits<double>::infinity();
        return false;
    }

    kd_tree_t target_kdtree(3, target, 10);

    auto normals_source = computeNormals(source);
    std::vector<Eigen::Matrix3d> covariances_source;
    covariances_source.resize(normals_source.size());
    auto normals_target = computeNormalskdTree(target, target_kdtree);
    std::vector<Eigen::Matrix3d> covariances_target;
    covariances_target.resize(normals_target.size());
    const Eigen::Matrix3d C = Eigen::Vector3d(epsilon_, 1, 1).asDiagonal();

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, normals_source.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                const auto Rx = GetRotationFromE1ToX(normals_source[i]);
                covariances_source[i] = Rx * C * Rx.transpose();
            }
        }
    );

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, normals_target.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                const auto Rx = GetRotationFromE1ToX(normals_target[i]);
                covariances_target[i] = Rx * C * Rx.transpose();
            }
        }
    );

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    int num_it = 0;
    
    // Search correspondences
    std::vector<Eigen::Vector2i> correspondences; // (source_index, target_index)
    correspondences.reserve(source.size());
    double error2 = 0.0; // sum of squared distances
    if (max_correspondence_distance > 0.0 && !source.empty() && !target.empty()) {
        find_correspondences_and_calculate_error(source, correspondences, target_kdtree, max_correspondence_distance, error2);
    } else {
        ROS_WARN_STREAM(source.size() << ", " << target.size() << ", " << max_correspondence_distance);
        ROS_WARN_STREAM("Registration not possible!");
        return false;
    }
    double fitness = 0.0;
    double inlier_rmse = 0.0, prev_rmse = 0.0, prev_prev_rmse = 0.0;
    if (!correspondences.empty()) {
        fitness = static_cast<double>(correspondences.size()) / static_cast<double>(source.size());
        inlier_rmse = std::sqrt(error2 / static_cast<double>(correspondences.size()));
    } else {
        fitness = 0.0;
        inlier_rmse = 0.0;
    }

    /* ICP MAIN LOOP */
    for (int j = 0; j < max_num_iterations; ++j)
    {
        prev_prev_rmse = prev_rmse;
        prev_rmse = inlier_rmse;

        Sophus::SE3d estimation;
        bool valid = true;
        double dx_norm;
        if (correspondences.empty() || covariances_source.empty() || covariances_target.empty()) {
            estimation = Sophus::SE3d();
            valid = false;
        } else {
            // Build linear system (GICP style) including robust weights
            const auto &[JTJ, JTr, residual_sum] = BuildLinearSystemGICP(source, target, covariances_source, covariances_target, correspondences, kernel);
            const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
            Eigen::Matrix4d output;
            output.setIdentity();
            output.block<3, 3>(0, 0) =
                (Eigen::AngleAxisd(dx(2), Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(dx(1), Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(dx(0), Eigen::Vector3d::UnitX())).matrix();
            output.block<3, 1>(0, 3) = dx.block<3, 1>(3, 0);
            estimation = Sophus::SE3d::fitToSE3(output);
        }
        transformPoints(estimation, source);
        transformCovariances(estimation, covariances_source);
        error2 = 0.0;
        find_correspondences_and_calculate_error(source, correspondences, target_kdtree, max_correspondence_distance, error2);
        double prev_fitness = fitness;
        if (!correspondences.empty()) {
            fitness = static_cast<double>(correspondences.size()) / static_cast<double>(source.size());
            inlier_rmse = std::sqrt(error2 / static_cast<double>(correspondences.size()));
        } else {
            fitness = 0.0;
            inlier_rmse = 0.0;
        }
        if (config_.print_runtime) {
            ROS_INFO_STREAM("RMSE: " << inlier_rmse 
                << " (cond1: " << std::fabs(inlier_rmse - prev_rmse) 
                << "), (cond2: " << std::fabs(inlier_rmse - prev_prev_rmse) 
                << ") should be < " << config_.gicp_th_rmse);
        }
        T_icp = estimation * T_icp;
        num_it = j + 1;
        // 3DTK style termination condition
        if ((std::fabs(inlier_rmse - prev_rmse) < config_.gicp_th_rmse &&
             std::fabs(inlier_rmse - prev_prev_rmse) < config_.gicp_th_rmse) || 
             j > max_num_iterations)
        {
            // TODO: Consider fitness in termination condition again!! 
            break;
        }
    }

    double gicp_elapsed = std::chrono::duration<double, std::milli>
        (std::chrono::high_resolution_clock::now() - gicp_t0).count();

    registered_pose = T_icp * initial_guess;
    matching_error = inlier_rmse;
    if (config_.print_runtime || gicp_elapsed >= 50.0) { 
        ROS_INFO_STREAM("Performed " << num_it << " iterations in " << gicp_elapsed << " ms");
    }
    return true;
}

std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> LIONode::BuildLinearSystemGICP(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    const std::vector<Eigen::Matrix3d> &c_source,
    const std::vector<Eigen::Matrix3d> &c_target,
    const std::vector<Eigen::Vector2i> &corres,
    double kernel)
{
    // Another function defintion
    auto compute_jacobian_and_residual =
        [&](int i,
            std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &J_r,
            std::vector<double> &r, std::vector<double> &w)
    {
        const Eigen::Vector3d &vs = source[corres[i][0]];
        const Eigen::Matrix3d &Cs = c_source[corres[i][0]];
        const Eigen::Vector3d &vt = target[corres[i][1]];
        const Eigen::Matrix3d &Ct = c_target[corres[i][1]];
        const Eigen::Vector3d d = vs - vt;
        const Eigen::Matrix3d M = Ct + Cs;
        const Eigen::Matrix3d W = computeInverseSqrtSPD(M);

        Eigen::Matrix<double, 3, 6> J;
        Eigen::Matrix3d vs_skew;
        vs_skew << 0, -vs.z(), vs.y(),
            vs.z(), 0, -vs.x(),
            -vs.y(), vs.x(), 0;
        J.block<3, 3>(0, 0) = -vs_skew;
        J.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        J = W * J;

        constexpr int n_rows = 3;
        J_r.resize(n_rows);
        r.resize(n_rows);
        w.resize(n_rows);
        for (size_t i = 0; i < n_rows; ++i)
        {
            r[i] = W.row(i).dot(d);
            if (config_.robust_kernel)
            {
                w[i] = kernel / square(kernel + square(r[i]));
            }
            else
            {
                w[i] = 1.0;
            }
            J_r[i] = J.row(i);
        }
    };
    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    double r2 = -1.0;
    std::tie(JTJ, JTr, r2) = ComputeJTJandJTr(compute_jacobian_and_residual, (int)corres.size());

    // Return the accumulated residual sum (variable name fixed from r2_sum to r2)
    return std::make_tuple(JTJ, JTr, r2);
}

std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double> LIONode::ComputeJTJandJTr(
    std::function<
        void(int,
             std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &,
             std::vector<double> &,
             std::vector<double> &)>
        f,
    int iteration_num)
{
    if (iteration_num <= 0)
    {
        return std::make_tuple(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0);
    }

    struct Accumulator
    {
        Eigen::Matrix6d JTJ;
        Eigen::Vector6d JTr;
        double r2_sum;
        Accumulator() : JTJ(Eigen::Matrix6d::Zero()), JTr(Eigen::Vector6d::Zero()), r2_sum(0.0) {}
    };

    Accumulator result = tbb::parallel_reduce(
        tbb::blocked_range<int>(0, iteration_num),
        Accumulator{},
        [&](const tbb::blocked_range<int> &range, Accumulator acc) -> Accumulator
        {
            std::vector<double> r;                                                       // residuals per call
            std::vector<double> w;                                                       // weights per residual row
            std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> J_r; // Jacobian rows
            for (int i = range.begin(); i < range.end(); ++i)
            {
                f(i, J_r, r, w);
                const int rows = static_cast<int>(r.size());
                for (int j = 0; j < rows; j++)
                {
                    acc.JTJ.noalias() += J_r[j] * w[j] * J_r[j].transpose();
                    acc.JTr.noalias() += J_r[j] * w[j] * r[j];
                    acc.r2_sum += r[j] * r[j];
                }
            }
            return acc;
        },
        [](Accumulator a, const Accumulator &b) -> Accumulator
        {
            a.JTJ += b.JTJ;
            a.JTr += b.JTr;
            a.r2_sum += b.r2_sum;
            return a;
        });
    return std::make_tuple(std::move(result.JTJ), std::move(result.JTr), result.r2_sum);
}

std::vector<Eigen::Matrix3d> LIONode::computePerPointCovariances(const std::vector<Eigen::Vector3d> &points)
{
    size_t k_neighbors = 20;
    std::vector<Eigen::Matrix3d> covariances;
    covariances.resize(points.size());

    kd_tree_t kdtree(3, points, 10);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            // thread-local buffers per range to avoid contention
            std::vector<size_t> indices(k_neighbors);
            std::vector<double> dists(k_neighbors);
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                const double query_pt[3] = {points[i].x(), points[i].y(), points[i].z()};
                size_t found = kdtree.index->knnSearch(query_pt, k_neighbors, indices.data(), dists.data());
                if (found >= 3) {
                    covariances[i] = computeCovariance(points, indices.data(), found);
                } else {
                    covariances[i] = Eigen::Matrix3d::Identity();
                }
            }
        }
    );
    return covariances;
}

std::vector<Eigen::Matrix3d> LIONode::computePerPointCovarianceskdTree(const std::vector<Eigen::Vector3d> &points,
                                                                       const kd_tree_t &kdtree)
{
    size_t k_neighbors = 20;
    std::vector<Eigen::Matrix3d> covariances;
    covariances.resize(points.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            // thread-local buffers per range to avoid contention
            std::vector<size_t> indices(k_neighbors);
            std::vector<double> dists(k_neighbors);
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                const double query_pt[3] = {points[i].x(), points[i].y(), points[i].z()};
                size_t found = kdtree.index->knnSearch(query_pt, k_neighbors, indices.data(), dists.data());
                if (found >= 3)
                {
                    covariances[i] = computeCovariance(points, indices.data(), found);
                }
                else
                {
                    covariances[i] = Eigen::Matrix3d::Identity();
                }
            }
        }
    );
    return covariances;
}

// Cumulant-based covariance of neighbour points given indices[0..count-1]
inline Eigen::Matrix3d LIONode::computeCovariance(const std::vector<Eigen::Vector3d> &points,
                                                  const size_t *indices,
                                                  size_t count)
{
    if (count < 3)
        return Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 9, 1> cumulants;
    cumulants.setZero();
    for (size_t j = 0; j < count; ++j)
    {
        const Eigen::Vector3d &p = points[indices[j]];
        cumulants(0) += p.x();
        cumulants(1) += p.y();
        cumulants(2) += p.z();
        cumulants(3) += p.x() * p.x();
        cumulants(4) += p.x() * p.y();
        cumulants(5) += p.x() * p.z();
        cumulants(6) += p.y() * p.y();
        cumulants(7) += p.y() * p.z();
        cumulants(8) += p.z() * p.z();
    }
    cumulants /= static_cast<double>(count);
    Eigen::Matrix3d C;
    C(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
    C(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
    C(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
    C(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
    C(1, 0) = C(0, 1);
    C(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
    C(2, 0) = C(0, 2);
    C(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
    C(2, 1) = C(1, 2);
    return C;
}

/** @brief Computes the eigenvector corresponding to eigenvalue eval0 of a symmetric 3x3 matrix
 *         by selecting the cross product of the two rows of (A - eval0*I) with largest norm.
 * @param A Symmetric 3x3 matrix.
 * @param eval0 The eigenvalue whose eigenvector is sought.
 * @return Unit-length eigenvector associated with eval0.
 */
static inline Eigen::Vector3d ComputeEigenvector0(const Eigen::Matrix3d &A, double eval0)
{
    Eigen::Vector3d row0(A(0, 0) - eval0, A(0, 1), A(0, 2));
    Eigen::Vector3d row1(A(0, 1), A(1, 1) - eval0, A(1, 2));
    Eigen::Vector3d row2(A(0, 2), A(1, 2), A(2, 2) - eval0);
    Eigen::Vector3d r0xr1 = row0.cross(row1);
    Eigen::Vector3d r0xr2 = row0.cross(row2);
    Eigen::Vector3d r1xr2 = row1.cross(row2);
    double d0 = r0xr1.dot(r0xr1);
    double d1 = r0xr2.dot(r0xr2);
    double d2 = r1xr2.dot(r1xr2);
    double dmax = d0;
    int imax = 0;
    if (d1 > dmax)
    {
        dmax = d1;
        imax = 1;
    }
    if (d2 > dmax)
    {
        imax = 2;
    }
    if (imax == 0)
        return r0xr1 / std::sqrt(d0);
    if (imax == 1)
        return r0xr2 / std::sqrt(d1);
    return r1xr2 / std::sqrt(d2);
}

/** @brief Computes the eigenvector for eigenvalue eval1 of a symmetric 3x3 matrix,
 *         given a known eigenvector evec0 for a different eigenvalue. Uses the 2D
 *         reduced problem in the plane orthogonal to evec0.
 * @param A Symmetric 3x3 matrix.
 * @param evec0 A previously computed eigenvector of A.
 * @param eval1 The eigenvalue whose eigenvector is sought.
 * @return Unit-length eigenvector associated with eval1.
 */
static inline Eigen::Vector3d ComputeEigenvector1(const Eigen::Matrix3d &A,
                                                  const Eigen::Vector3d &evec0,
                                                  double eval1)
{
    Eigen::Vector3d U, V;
    if (std::abs(evec0(0)) > std::abs(evec0(1)))
    {
        double inv_length = 1.0 / std::sqrt(evec0(0) * evec0(0) + evec0(2) * evec0(2));
        U << -evec0(2) * inv_length, 0, evec0(0) * inv_length;
    }
    else
    {
        double inv_length = 1.0 / std::sqrt(evec0(1) * evec0(1) + evec0(2) * evec0(2));
        U << 0, evec0(2) * inv_length, -evec0(1) * inv_length;
    }
    V = evec0.cross(U);
    Eigen::Vector3d AU(A(0, 0) * U(0) + A(0, 1) * U(1) + A(0, 2) * U(2),
                       A(0, 1) * U(0) + A(1, 1) * U(1) + A(1, 2) * U(2),
                       A(0, 2) * U(0) + A(1, 2) * U(1) + A(2, 2) * U(2));
    Eigen::Vector3d AV(A(0, 0) * V(0) + A(0, 1) * V(1) + A(0, 2) * V(2),
                       A(0, 1) * V(0) + A(1, 1) * V(1) + A(1, 2) * V(2),
                       A(0, 2) * V(0) + A(1, 2) * V(1) + A(2, 2) * V(2));

    double m00 = U(0) * AU(0) + U(1) * AU(1) + U(2) * AU(2) - eval1;
    double m01 = U(0) * AV(0) + U(1) * AV(1) + U(2) * AV(2);
    double m11 = V(0) * AV(0) + V(1) * AV(1) + V(2) * AV(2) - eval1;
    double absM00 = std::abs(m00), absM01 = std::abs(m01), absM11 = std::abs(m11);
    double max_abs_comp;
    if (absM00 >= absM11)
    {
        max_abs_comp = std::max(absM00, absM01);
        if (max_abs_comp > 0)
        {
            if (absM00 >= absM01)
            {
                m01 /= m00;
                m00 = 1 / std::sqrt(1 + m01 * m01);
                m01 *= m00;
            }
            else
            {
                m00 /= m01;
                m01 = 1 / std::sqrt(1 + m00 * m00);
                m00 *= m01;
            }
            return m01 * U - m00 * V;
        }
        else
        {
            return U;
        }
    }
    else
    {
        max_abs_comp = std::max(absM11, absM01);
        if (max_abs_comp > 0)
        {
            if (absM11 >= absM01)
            {
                m01 /= m11;
                m11 = 1 / std::sqrt(1 + m01 * m01);
                m01 *= m11;
            }
            else
            {
                m11 /= m01;
                m01 = 1 / std::sqrt(1 + m11 * m11);
                m11 *= m01;
            }
            return m11 * U - m01 * V;
        }
        else
        {
            return U;
        }
    }
}

/** @brief Performs a fast closed-form eigen decomposition of a symmetric 3x3 matrix
 *         and returns the eigenvector with the smallest eigenvalue (surface normal direction).
 * @param covariance Symmetric 3x3 covariance matrix.
 * @return Unit eigenvector corresponding to the smallest eigenvalue.
 */
static inline Eigen::Vector3d FastEigen3x3(const Eigen::Matrix3d &covariance)
{
    Eigen::Matrix3d A = covariance;
    double max_coeff = A.maxCoeff();
    if (max_coeff == 0)
        return Eigen::Vector3d::Zero();
    A /= max_coeff;
    double norm = A(0, 1) * A(0, 1) + A(0, 2) * A(0, 2) + A(1, 2) * A(1, 2);
    if (norm > 0)
    {
        Eigen::Vector3d eval;
        Eigen::Vector3d evec0, evec1, evec2;
        double q = (A(0, 0) + A(1, 1) + A(2, 2)) / 3.0;
        double b00 = A(0, 0) - q, b11 = A(1, 1) - q, b22 = A(2, 2) - q;
        double p = std::sqrt((b00 * b00 + b11 * b11 + b22 * b22 + 2 * norm) / 6.0);
        double c00 = b11 * b22 - A(1, 2) * A(1, 2);
        double c01 = A(0, 1) * b22 - A(1, 2) * A(0, 2);
        double c02 = A(0, 1) * A(1, 2) - b11 * A(0, 2);
        double det = (b00 * c00 - A(0, 1) * c01 + A(0, 2) * c02) / (p * p * p);
        double half_det = det * 0.5;
        half_det = std::min(std::max(half_det, -1.0), 1.0);
        double angle = std::acos(half_det) / 3.0;
        const double two_thirds_pi = 2.09439510239319549;
        double beta2 = std::cos(angle) * 2.0;
        double beta0 = std::cos(angle + two_thirds_pi) * 2.0;
        double beta1 = -(beta0 + beta2);
        eval(0) = q + p * beta0;
        eval(1) = q + p * beta1;
        eval(2) = q + p * beta2;
        if (half_det >= 0)
        {
            evec2 = ComputeEigenvector0(A, eval(2));
            if (eval(2) < eval(0) && eval(2) < eval(1))
            {
                A *= max_coeff;
                return evec2;
            }
            evec1 = ComputeEigenvector1(A, evec2, eval(1));
            A *= max_coeff;
            if (eval(1) < eval(0) && eval(1) < eval(2))
                return evec1;
            evec0 = evec1.cross(evec2);
            return evec0;
        }
        else
        {
            evec0 = ComputeEigenvector0(A, eval(0));
            if (eval(0) < eval(1) && eval(0) < eval(2))
            {
                A *= max_coeff;
                return evec0;
            }
            evec1 = ComputeEigenvector1(A, evec0, eval(1));
            A *= max_coeff;
            if (eval(1) < eval(0) && eval(1) < eval(2))
                return evec1;
            evec2 = evec0.cross(evec1);
            return evec2;
        }
    }
    else
    {
        A *= max_coeff;
        if (A(0, 0) < A(1, 1) && A(0, 0) < A(2, 2))
            return Eigen::Vector3d(1, 0, 0);
        if (A(1, 1) < A(0, 0) && A(1, 1) < A(2, 2))
            return Eigen::Vector3d(0, 1, 0);
        return Eigen::Vector3d(0, 0, 1);
    }
}

std::vector<Eigen::Vector3d> LIONode::computeNormals(const std::vector<Eigen::Vector3d> &points)
{
    auto covs = computePerPointCovariances(points);
    std::vector<Eigen::Vector3d> normals(points.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                Eigen::Vector3d n = FastEigen3x3(covs[i]);
                double len = n.norm();
                if (len < 1e-9)
                    n = Eigen::Vector3d::UnitZ();
                // Orient normal toward sensor origin (flip if pointing away from origin based on point direction)
                // if (n.dot(points[i]) > 0) n = -n;
                normals[i] = n;
            }
        }
    );
    return normals;
}

std::vector<Eigen::Vector3d> LIONode::computeNormalskdTree(const std::vector<Eigen::Vector3d> &points, const kd_tree_t &kdtree)
{
    auto covs = computePerPointCovarianceskdTree(points, kdtree);
    std::vector<Eigen::Vector3d> normals(points.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                Eigen::Vector3d n = FastEigen3x3(covs[i]);
                double len = n.norm();
                if (len < 1e-9) {
                    n = Eigen::Vector3d::UnitZ();
                }
                // Orient normal toward sensor origin (flip if pointing away from origin based on point direction)
                if (n.dot(points[i]) > 0) n = -n;
                normals[i] = n;
            }
        }
    );
    return normals;
}
