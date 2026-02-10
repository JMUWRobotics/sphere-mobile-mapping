/*
 * Implementation of global map-based ground normal finder
 * Based on ground_finder.cpp by Carolin Bösch
 * Adapted for Moritz FIXME's globally consistent mapping node
 */

#include "global_ground_finder.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>
#include <fstream>
#include <iomanip>

GlobalGroundFinder::GlobalGroundFinder(ros::NodeHandle &nh, ros::NodeHandle &pnh, PlaneSegm plane_algorithm)
    : nh(nh), pnh(pnh),
      plane_algorithm_(plane_algorithm),
      map_received_(false),
      pose_received_(false),
      count_success_(0),
      count_fail_(0),
      last_inlier_count_(0),
      last_local_cloud_size_(0),
      last_roll_(0.0),
      last_pitch_(0.0)
{
    // Initialize point cloud and kdtree
    global_map_.reset(new pcl::PointCloud<PointType>);
    kdtree_.reset(new pcl::KdTreeFLANN<PointType>);

    n_ = {0.0, 0.0, -1.0};

    // normal computation params
    pnh.param<bool>("quiet", quiet_, false);
    pnh.param<bool>("enable_scoring", enable_scoring_, true);
    pnh.param<double>("extraction_radius", extraction_radius_, 3.0);
    pnh.param<double>("extraction_height", extraction_height_, 0.5);
    pnh.param<double>("min_points_for_plane", min_points_for_plane_, 10.0);
    pnh.param<double>("wall_threshold", wall_threshold_, 0.707); // cos(45°)
    pnh.param<double>("score_threshold", score_threshold_, 0.2);
    pnh.param<double>("min_score_window", min_score_window_, 0.3);
    pnh.param<double>("weight_visibility", weight_visibility_, 0.6);
    pnh.param<double>("weight_inlier_ratio", weight_inlier_ratio_, 0.4);
    pnh.param<double>("min_inliers", min_inliers_, 20.0);
    pnh.param<double>("inlier_scale", inlier_scale_, 0.3);
    pnh.param<int>("max_iterations_plane_detection", max_iterations_plane_detection_, 3);

    // Smoothing params
    pnh.param<bool>("enable_normal_smoothing", enable_normal_smoothing_, true);
    pnh.param<double>("normal_smoothing_alpha", normal_smoothing_alpha_, 0.1);
    pnh.param<bool>("use_gaussian_smoothing", use_gaussian_smoothing_, false);
    pnh.param<double>("smoothing_cutoff_freq", smoothing_cutoff_freq_, 0.2);
    pnh.param<double>("update_rate", update_rate_, 20.0);
    have_smoothed_normal_ = false;

    if (use_gaussian_smoothing_)
    {
        float dt = 1.0f / static_cast<float>(update_rate_);
        float sigma = 1.0f / (2.0f * M_PI * static_cast<float>(smoothing_cutoff_freq_));
        size_t win_size = 6.0f * sigma / dt;
        win_size = win_size % 2 == 0 ? win_size + 1 : win_size;
        gaussian_kernel_.reset(new SmoothedGaussian3D(win_size, sigma, dt));
        ROS_INFO("created gaussian kernel smoothing with \n window size: %ld, sigma: %.4f s, dt: %.4f s, cutoff freq: %.4f Hz, update_rate: %.4f Hz", win_size, sigma, dt, smoothing_cutoff_freq_, update_rate_);
    }

    std::string filename;
    pnh.param<std::string>("file", filename, "default");
    write2file = (filename != "default");

    if (write2file)
    {
        std::string alg_prefix;
        switch (plane_algorithm_)
        {
        case PCA:
            alg_prefix = "pca";
            break;
        case RANSAC:
            alg_prefix = "ransac";
            break;
        case RHT:
            alg_prefix = "rht";
            break;
        case RHT2:
            alg_prefix = "rht2";
            break;
        }

        std::string package_path = ros::package::getPath("global_ground_finder");
        log_file_path_ = package_path + "/data/" + alg_prefix + "/" + filename + ".csv";

        log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
        if (log_file_.is_open())
        {
            log_file_ << "timestamp,nx,ny,nz,visibility_score,inlier_score,combined_score\n";
            log_file_.flush();
            ROS_INFO("Logging to file: %s", log_file_path_.c_str());
        }
        else
        {
            ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
            write2file = false;
        }
    }
    else
    {
        ROS_INFO("File logging disabled (use 'file' parameter to enable)");
    }

    sub_map = nh.subscribe("/map_out", 10, &GlobalGroundFinder::mapCallback, this);
    sub_pose = nh.subscribe("/lkf/pose", 10, &GlobalGroundFinder::poseCallback, this);

    pub_local_cloud = nh.advertise<sensor_msgs::PointCloud2>("/global_ground_finder/local_cloud", 1);
    pub_inliers = nh.advertise<sensor_msgs::PointCloud2>("/global_ground_finder/inliers", 1);
    pub_normal = nh.advertise<geometry_msgs::Vector3Stamped>("/global_ground_finder/normal", 1);
    pub_scored_normal = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("/global_ground_finder/scored_normal", 1);
    pub_smoothed_normal = nh.advertise<geometry_msgs::Vector3Stamped>("/global_ground_finder/smoothed_normal", 1);
    pub_smoothed_scored_normal = nh.advertise<geometry_msgs::Vector3Stamped>("/global_ground_finder/smoothed_scored_normal", 1);
    pub_normal_marker = nh.advertise<visualization_msgs::Marker>("/global_ground_finder/normal_marker", 1);

    initMarker();

    ROS_INFO("Global Ground Finder initialized");
    ROS_INFO("  Extraction radius: %.2f m", extraction_radius_);
    ROS_INFO("  Extraction height: %.2f m", extraction_height_);
    ROS_INFO("  Enable scoring: %s", enable_scoring_ ? "true" : "false");
    ROS_INFO("  Enable smoothing: %s (%s)",
             enable_normal_smoothing_ ? "true" : "false",
             use_gaussian_smoothing_ ? "Gaussian" : "EMA");
}

GlobalGroundFinder::~GlobalGroundFinder()
{
    if (log_file_.is_open())
    {
        log_file_.close();
        ROS_INFO("Closed log file: %s", log_file_path_.c_str());
    }
    ROS_INFO("Global Ground Finder shutting down");
    ROS_INFO("  Success: %d, Failures: %d", count_success_, count_fail_);
}

void GlobalGroundFinder::initMarker()
{
    normal_marker_.type = visualization_msgs::Marker::ARROW;
    normal_marker_.action = visualization_msgs::Marker::ADD;
    normal_marker_.pose.orientation.w = 1.0;
    normal_marker_.scale.x = 0.02;
    normal_marker_.scale.y = 0.06;
    normal_marker_.scale.z = 0.06;
    normal_marker_.color.a = 1.0;
    normal_marker_.color.r = 0.0;
    normal_marker_.color.g = 1.0;
    normal_marker_.color.b = 0.0;
    normal_marker_.header.frame_id = "map3";
}

void GlobalGroundFinder::mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!quiet_)
    {
        ROS_INFO("Received global map: %d points", msg->width * msg->height);
    }

    // Convert to PCL
    pcl::fromROSMsg(*msg, *global_map_);

    if (global_map_->points.size() < min_points_for_plane_)
    {
        ROS_WARN("Global map too small (%zu points)", global_map_->points.size());
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    kdtree_->setInputCloud(global_map_);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    map_received_ = true;
    map_timestamp_ = msg->header.stamp;

    if (!quiet_)
    {
        ROS_INFO("Built KdTree in %ld ms", duration);
    }

    if (pose_received_)
    {
        // always block on processing
        std::lock_guard<std::mutex> lock(processing_mutex_);
        processAtCurrentPose();
    }
}

void GlobalGroundFinder::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Update current pose with protection
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = *msg; // pose from odom_frame to map2/3? frame (kugelmittelpunkt) -- zu pandar nötig -> map3 to pandar; anfangs odom->pandar static transform und dann multiply with msg here
        pose_received_ = true;
    }

    if (map_received_)
    {
        // try to process only if not already busy
        std::unique_lock<std::mutex> lock(processing_mutex_, std::try_to_lock);
        if (lock.owns_lock())
        {
            processAtCurrentPose();
        }
        // else skip this pose update to avoid mutex lock blocking
    }
}

void GlobalGroundFinder::processAtCurrentPose()
{
    auto start_total = std::chrono::high_resolution_clock::now();

    // Copy for thread safety
    geometry_msgs::PoseStamped pose_copy;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_copy = current_pose_;
    }

    // get local cloud around curr pose
    pcl::PointCloud<PointType>::Ptr local_cloud(new pcl::PointCloud<PointType>);
    if (!extractLocalCloud(pose_copy, local_cloud))
    {
        ROS_WARN("Failed to extract local cloud");
        count_fail_++;
        return;
    }

    last_local_cloud_size_ = local_cloud->points.size();

    sensor_msgs::PointCloud2 local_msg;
    pcl::toROSMsg(*local_cloud, local_msg);
    local_msg.header.stamp = pose_copy.header.stamp;
    local_msg.header.frame_id = "map3";
    pub_local_cloud.publish(local_msg);

    // calc ground plane
    std::vector<double> normal;
    size_t inlier_count = 0;
    if (!fitGroundPlane(local_cloud, normal, inlier_count))
    {
        ROS_WARN("Failed to fit ground plane");
        count_fail_++;
        return;
    }

    last_inlier_count_ = inlier_count;

    auto end_total = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count();

    if (!quiet_)
    {
        ROS_INFO("Ground normal computed in %.2f ms: [%.3f, %.3f, %.3f], inliers: %zu/%zu",
                 duration / 1000.0, normal[0], normal[1], normal[2],
                 inlier_count, local_cloud->points.size());
    }

    double vis_score = 1.0;
    double inlier_score = 1.0;
    double combined_score = 1.0;

    if (enable_scoring_)
    {
        auto scores = compute_scores(pose_copy, inlier_count, local_cloud->points.size());
        vis_score = scores.first;
        inlier_score = scores.second;
        combined_score = combine_scores(vis_score, inlier_score);
    }

    ground_finder_msgs::ScoredNormalStamped scored_msg;
    scored_msg.header = pose_copy.header; // use pose timestamp for scored normal TOOD: check which frame. map3 or map2
    scored_msg.normal.x = normal[0];
    scored_msg.normal.y = normal[1];
    scored_msg.normal.z = normal[2];
    scored_msg.visibility_score = vis_score;
    scored_msg.inlier_score = inlier_score;
    scored_msg.combined_score = combined_score;

    if (write2file)
    {
        log_results(pose_copy.header.stamp, normal, vis_score, inlier_score, combined_score);
    }

    // Add to sliding window
    if (enable_scoring_)
    {
        scored_window_.push_back(scored_msg);
        if (scored_window_.size() > MAX_WINDOW_SIZE)
        {
            scored_window_.pop_front();
        }
    }

    // Check for fallback
    bool using_fallback = false;
    if (enable_scoring_ && combined_score < score_threshold_)
    {
        ground_finder_msgs::ScoredNormalStamped fallback_msg;
        std::vector<double> fallback_normal;

        if (find_fallback_normal(combined_score, fallback_normal, fallback_msg))
        {
            normal = fallback_normal;
            scored_msg = fallback_msg;
            using_fallback = true;
            ROS_WARN("Using fallback normal (score: %.3f)", fallback_msg.combined_score);
        }
        else
        {
            ROS_WARN("No suitable fallback found");
        }
    }

    // Publish stuff
    geometry_msgs::Vector3Stamped n_msg;
    n_msg.header = pose_copy.header;
    n_msg.vector.x = normal[0];
    n_msg.vector.y = normal[1];
    n_msg.vector.z = normal[2];
    pub_normal.publish(n_msg);
    pub_scored_normal.publish(scored_msg);
    publish_normal_marker(normal, pose_copy.header.stamp);

    if (enable_normal_smoothing_)
    {
        // Smooth raw normal
        geometry_msgs::Vector3Stamped smoothed_raw;
        if (use_gaussian_smoothing_)
        {
            smoothed_raw = gaussian_smoothing(n_msg);
        }
        else
        {
            smoothed_raw = ema_smoothing(n_msg);
        }
        smoothed_raw.header.stamp = n_msg.header.stamp;
        smoothed_raw.header.frame_id = n_msg.header.frame_id;
        pub_smoothed_normal.publish(smoothed_raw);

        geometry_msgs::Vector3Stamped scored_vec;
        scored_vec.header = scored_msg.header;
        scored_vec.vector = scored_msg.normal;

        geometry_msgs::Vector3Stamped smoothed_scored;
        if (use_gaussian_smoothing_)
        {
            smoothed_scored = gaussian_smoothing(scored_vec);
        }
        else
        {
            smoothed_scored = ema_smoothing(scored_vec);
        }
        smoothed_scored.header.stamp = scored_msg.header.stamp;
        smoothed_scored.header.frame_id = scored_msg.header.frame_id;
        pub_smoothed_scored_normal.publish(smoothed_scored);
    }

    // internal state uodating
    n_ = normal;
    count_success_++;
}

/*  ------------------------------------------
    Functions called in processAtCurrentPose()
    ------------------------------------------
*/
bool GlobalGroundFinder::extractLocalCloud(const geometry_msgs::PoseStamped &pose,
                                           pcl::PointCloud<PointType>::Ptr &local_cloud)
{
    if (!map_received_ || global_map_->points.size() == 0)
    {
        return false;
    }

    PointType query_point;
    query_point.x = pose.pose.position.x;
    query_point.y = pose.pose.position.y;
    query_point.z = pose.pose.position.z;

    // search in radius around curr pose
    std::vector<int> indices;
    std::vector<float> distances;

    if (kdtree_->radiusSearch(query_point, extraction_radius_, indices, distances) == 0)
    {
        return false;
    }

    // Filter by height relative to curr z
    local_cloud->points.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const auto &pt = global_map_->points[indices[i]];
        double height_diff = std::abs(pt.z - query_point.z); // vertical dist to curr pose's z component

        if (height_diff <= extraction_height_) // only keep the points within +- 0.5m vertically
        {
            local_cloud->points.push_back(pt);
        }
    }

    local_cloud->width = local_cloud->points.size();
    local_cloud->height = 1; // tells pcl it's "unorganized" so no rows/column structure like camera data
    local_cloud->is_dense = true;

    return local_cloud->points.size() >= min_points_for_plane_;
}

bool GlobalGroundFinder::fitGroundPlane(const pcl::PointCloud<PointType>::Ptr &local_cloud,
                                        std::vector<double> &normal,
                                        size_t &inlier_count)
{
    bool success = false;

    switch (plane_algorithm_)
    {
    case PCA:
        success = fitPlanePCA(local_cloud, normal);
        inlier_count = local_cloud->points.size(); // (PCA uses all points)
        break;
    case RANSAC:
        success = fitPlaneRANSAC(local_cloud, normal, inlier_count);
        break;
    case RHT:
        success = fitPlaneRHT(local_cloud, normal, inlier_count);
        break;
    case RHT2:
        success = fitPlaneRHT2(local_cloud, normal, inlier_count);
        break;
    }

    if (!success)
    {
        return false;
    }

    // Validate it's ground (not wall)
    return validateGroundNormal(normal);
}

bool GlobalGroundFinder::fitPlanePCA(const pcl::PointCloud<PointType>::Ptr &cloud,
                                     std::vector<double> &normal)
{
    if (cloud->points.size() < 3)
    {
        return false;
    }

    try
    {
        pcl::PCA<PointType> pca;
        pca.setInputCloud(cloud);
        Eigen::Matrix3f eigen_vecs = pca.getEigenVectors();

        // Normal is smallest eigenvector (3rd column)
        normal.resize(3);
        normal[0] = eigen_vecs(0, 2);
        normal[1] = eigen_vecs(1, 2);
        normal[2] = eigen_vecs(2, 2);

        return true;
    }
    catch (...)
    {
        ROS_ERROR("PCA failed");
        return false;
    }
}

bool GlobalGroundFinder::fitPlaneRANSAC(const pcl::PointCloud<PointType>::Ptr &cloud,
                                        std::vector<double> &normal,
                                        size_t &inlier_count)
{
    if (cloud->points.size() < 3)
    {
        return false;
    }

    pcl::PointCloud<PointType>::Ptr cloud_work(new pcl::PointCloud<PointType>(*cloud));

    for (int iter = 0; iter < max_iterations_plane_detection_; ++iter)
    {
        try
        {
            // RANSAC
            pcl::SampleConsensusModelPlane<PointType>::Ptr model(
                new pcl::SampleConsensusModelPlane<PointType>(cloud_work));
            pcl::RandomSampleConsensus<PointType> ransac(model);
            ransac.setDistanceThreshold(0.05); // 5cm threshold
            ransac.computeModel();

            std::vector<int> inliers;
            ransac.getInliers(inliers);

            if (inliers.size() < 3)
            {
                return false;
            }

            // Use PCA on inliers for more stable normal
            pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
            inlier_indices->indices = inliers;

            pcl::PCA<PointType> pca;
            pca.setInputCloud(cloud_work);
            pca.setIndices(inlier_indices);
            Eigen::Matrix3f eigen_vecs = pca.getEigenVectors();

            normal.resize(3);
            normal[0] = eigen_vecs(0, 2);
            normal[1] = eigen_vecs(1, 2);
            normal[2] = eigen_vecs(2, 2);

            inlier_count = inliers.size();

            // Check if it's ground
            bool is_last = (iter == max_iterations_plane_detection_ - 1);
            std::vector<double> test_normal = normal;
            if (validateGroundNormal(test_normal))
            {
                normal = test_normal;
                return true;
            }
            else if (is_last)
            {
                return false;
            }

            // Remove inliers and try again (wall removal)
            pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
            cloud_filtered->points.reserve(cloud_work->points.size() - inliers.size());

            std::set<int> inlier_set(inliers.begin(), inliers.end());
            for (size_t i = 0; i < cloud_work->points.size(); ++i)
            {
                if (inlier_set.find(i) == inlier_set.end())
                {
                    cloud_filtered->points.push_back(cloud_work->points[i]);
                }
            }

            cloud_work = cloud_filtered;
            cloud_work->width = cloud_work->points.size();
            cloud_work->height = 1;

            if (cloud_work->points.size() < 3)
            {
                return false;
            }
        }
        catch (...)
        {
            return false;
        }
    }

    return false;
}

bool GlobalGroundFinder::fitPlaneRHT(const pcl::PointCloud<PointType>::Ptr &cloud,
                                     std::vector<double> &normal,
                                     size_t &inlier_count)
{
    if (cloud->points.size() < 3)
    {
        return false;
    }

    // Accumulator parameters
    int rhoNum = static_cast<int>(extraction_radius_) + 1;
    int phiNum = 180;
    int thetaNum = 360;
    int rhoMax = 1500;
    int accumulatorMax = 20;

    Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
    double minDist = 1.5;
    double maxDist = 50.0;

    pcl::PointCloud<PointType>::Ptr cloud_work(new pcl::PointCloud<PointType>(*cloud));

    for (int iter = 0; iter < max_iterations_plane_detection_; ++iter)
    {
        std::vector<double> n = {0.0, 0.0, 0.0};
        Hough hough(cloud_work, &acc, minDist, maxDist);
        double rho = hough.RHT(n);

        bool last_iteration = (iter == max_iterations_plane_detection_ - 1);

        if (rho != -1)
        {
            // Found a plane candidate, check if it's ground
            std::vector<double> test_normal = n;
            if (validateGroundNormal(test_normal))
            {
                normal = test_normal;

                // Count inliers
                inlier_count = 0;
                for (size_t i = 0; i < cloud_work->points.size(); ++i)
                {
                    const PointType &p = cloud_work->points[i];
                    double distance = std::abs(p.x * normal[0] + p.y * normal[1] + p.z * normal[2] - rho);
                    if (distance <= 0.05) // 5cm threshold
                    {
                        inlier_count++;
                    }
                }
                return true;
            }
            else if (last_iteration)
            {
                return false;
            }

            // Remove inliers (wall) and try again
            pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
            for (size_t i = 0; i < cloud_work->points.size(); ++i)
            {
                const PointType &p = cloud_work->points[i];
                double distance = std::abs(p.x * n[0] + p.y * n[1] + p.z * n[2] - rho);
                if (distance > 0.05)
                {
                    cloud_filtered->points.push_back(p);
                }
            }

            cloud_work = cloud_filtered;
            cloud_work->width = cloud_work->points.size();
            cloud_work->height = 1;

            if (cloud_work->points.size() < 3)
            {
                return false;
            }
        }
        else if (last_iteration)
        {
            return false;
        }
    }

    return false;
}

bool GlobalGroundFinder::fitPlaneRHT2(const pcl::PointCloud<PointType>::Ptr &cloud,
                                      std::vector<double> &normal,
                                      size_t &inlier_count)
{
    if (cloud->points.size() < 3)
    {
        return false;
    }

    // Accumulator parameters (lower accumulatorMax for rougher plane)
    int rhoNum = static_cast<int>(extraction_radius_) + 1;
    int phiNum = 180;
    int thetaNum = 360;
    int rhoMax = 1500;
    int accumulatorMax = 2;

    Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
    double minDist = 1.5;
    double maxDist = 50.0;

    pcl::PointCloud<PointType>::Ptr cloud_work(new pcl::PointCloud<PointType>(*cloud));

    for (int iter = 0; iter < max_iterations_plane_detection_; ++iter)
    {
        std::vector<double> temp_n = {0.0, 0.0, 0.0};
        Hough hough(cloud_work, &acc, minDist, maxDist);
        double rho = hough.RHT(temp_n);

        bool last_iteration = (iter == max_iterations_plane_detection_ - 1);

        if (rho != -1)
        {
            // Find inliers of rough plane
            std::vector<int> inliers;
            for (size_t i = 0; i < cloud_work->points.size(); ++i)
            {
                const PointType &p = cloud_work->points[i];
                double distance = std::abs(p.x * temp_n[0] + p.y * temp_n[1] + p.z * temp_n[2] - rho);
                if (distance <= 0.05)
                {
                    inliers.push_back(i);
                }
            }

            if (inliers.size() >= 3)
            {
                try
                {
                    // Refine with PCA on inliers
                    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
                    inlier_indices->indices = inliers;

                    pcl::PCA<PointType> pca;
                    pca.setInputCloud(cloud_work);
                    pca.setIndices(inlier_indices);
                    Eigen::Matrix3f eigen_vecs = pca.getEigenVectors();

                    std::vector<double> n(3);
                    n[0] = eigen_vecs(0, 2);
                    n[1] = eigen_vecs(1, 2);
                    n[2] = eigen_vecs(2, 2);

                    // Check if it's ground
                    std::vector<double> test_normal = n;
                    if (validateGroundNormal(test_normal))
                    {
                        normal = test_normal;
                        inlier_count = inliers.size();
                        return true;
                    }
                    else if (last_iteration)
                    {
                        return false;
                    }

                    // Remove inliers and try again
                    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
                    std::set<int> inlier_set(inliers.begin(), inliers.end());
                    for (size_t i = 0; i < cloud_work->points.size(); ++i)
                    {
                        if (inlier_set.find(i) == inlier_set.end())
                        {
                            cloud_filtered->points.push_back(cloud_work->points[i]);
                        }
                    }

                    cloud_work = cloud_filtered;
                    cloud_work->width = cloud_work->points.size();
                    cloud_work->height = 1;

                    if (cloud_work->points.size() < 3)
                    {
                        return false;
                    }
                }
                catch (...)
                {
                    if (last_iteration)
                    {
                        return false;
                    }
                    // Continue to next iteration
                }
            }
            else if (last_iteration)
            {
                return false;
            }
        }
        else if (last_iteration)
        {
            return false;
        }
    }

    return false;
}

bool GlobalGroundFinder::validateGroundNormal(std::vector<double> &normal)
{
    normalize_vector(normal);

    // Check against down vector [0, 0, -1]
    std::vector<double> down = {0.0, 0.0, -1.0};
    double dot = dot_product(normal, down);

    // Check if it's a wall (perpendicular to down)
    if (std::abs(dot) < wall_threshold_)
    {
        return false;
    }

    // Make sure it points downward
    if (dot < 0)
    {
        normal[0] = -normal[0];
        normal[1] = -normal[1];
        normal[2] = -normal[2];
    }

    return true;
}

std::pair<double, double> GlobalGroundFinder::compute_scores(const geometry_msgs::PoseStamped &pose,
                                                             size_t inlier_count,
                                                             size_t local_size)
{
    double visibility_score = 1.0;

    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    last_roll_ = roll;
    last_pitch_ = pitch;

    double roll_optim = std::abs(std::sin(2.0 * roll)); // Min at (0 and roll=+-90 deg), Max at (roll=+-45 deg)
    double roll_score = 0.1 + 0.9 * roll_optim;         // [0.1...1.0] to avoid zero visibility

    double pitch_optim = std::abs(std::sin(2.0 * pitch)); // Min at (0 and roll=+-90 deg), Max at (roll=+-45 deg)
    double pitch_score = 0.1 + 0.9 * pitch_optim;         // [0.1...1.0] to avoid zero visibility

    visibility_score = roll_score * pitch_score;

    // Inlier ratio score
    double inlier_score = 0.0;
    if (local_size > 0 && inlier_count >= min_inliers_)
    {
        double inlier_ratio = static_cast<double>(inlier_count) / static_cast<double>(local_size);
        inlier_score = std::min(std::max(inlier_ratio / inlier_scale_, 0.0), 1.0);
    }

    return std::make_pair(visibility_score, inlier_score);
}

double GlobalGroundFinder::combine_scores(double vis_score, double inlier_score)
{
    return weight_visibility_ * vis_score + weight_inlier_ratio_ * inlier_score;
}

bool GlobalGroundFinder::find_fallback_normal(double current_score,
                                              std::vector<double> &fallback_normal,
                                              ground_finder_msgs::ScoredNormalStamped &fallback_msg)
{
    if (scored_window_.empty())
    {
        return false;
    }

    // Find best score in window
    auto fallback = std::max_element(scored_window_.begin(), scored_window_.end(),
                                     [](const ground_finder_msgs::ScoredNormalStamped &a,
                                        const ground_finder_msgs::ScoredNormalStamped &b)
                                     {
                                         return a.combined_score < b.combined_score;
                                     });

    if (fallback != scored_window_.end() && fallback->combined_score >= min_score_window_)
    {
        fallback_msg = *fallback;
        fallback_normal = {fallback->normal.x, fallback->normal.y, fallback->normal.z};
        return true;
    }

    return false;
}

void GlobalGroundFinder::publish_normal_marker(const std::vector<double> &normal, const ros::Time &stamp)
{
    normal_marker_.header.stamp = stamp;
    normal_marker_.header.frame_id = "map3"; // TODO: check frame

    // Get current pose position under lock for marker visualization
    geometry_msgs::Point start;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_); // protect access to current_pose_
        start.x = current_pose_.pose.position.x;
        start.y = current_pose_.pose.position.y;
        start.z = current_pose_.pose.position.z;
    }

    geometry_msgs::Point end; // here no mutex lock needed since only dependent on start
    end.x = start.x + normal[0];
    end.y = start.y + normal[1];
    end.z = start.z + normal[2];

    normal_marker_.points.clear();
    normal_marker_.points.push_back(start);
    normal_marker_.points.push_back(end);

    pub_normal_marker.publish(normal_marker_);
}

geometry_msgs::Vector3Stamped GlobalGroundFinder::ema_smoothing(const geometry_msgs::Vector3Stamped &ground_vector)
{
    // Pass-through if disabled
    if (!enable_normal_smoothing_)
    {
        return ground_vector;
    }

    geometry_msgs::Vector3Stamped n = ground_vector;
    double n_x = n.vector.x;
    double n_y = n.vector.y;
    double n_z = n.vector.z;
    double n_norm = std::sqrt(n_x * n_x + n_y * n_y + n_z * n_z);

    if (n_norm > 1e-9)
    {
        n_x /= n_norm;
        n_y /= n_norm;
        n_z /= n_norm;
    }
    else
    {
        // Incoming vector invalid -> return existing smoothed normal if present
        if (have_smoothed_normal_)
            return smoothed_normal_;
        return ground_vector;
    }

    // For first call
    if (!have_smoothed_normal_)
    {
        smoothed_normal_.vector.x = n_x;
        smoothed_normal_.vector.y = n_y;
        smoothed_normal_.vector.z = n_z;
        have_smoothed_normal_ = true;
        return smoothed_normal_;
    }

    // ema
    double a = normal_smoothing_alpha_;    // [0,1]
    double px = smoothed_normal_.vector.x; // Prev. x
    double py = smoothed_normal_.vector.y; // Prev. y
    double pz = smoothed_normal_.vector.z; // Prev. z

    double sx = a * n_x + (1.0 - a) * px; // Smoothed x
    double sy = a * n_y + (1.0 - a) * py; // Smoothed y
    double sz = a * n_z + (1.0 - a) * pz; // Smoothed z

    double s_norm = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (s_norm > 1e-9) // Smoothed normal is valid
    {
        smoothed_normal_.vector.x = sx / s_norm;
        smoothed_normal_.vector.y = sy / s_norm;
        smoothed_normal_.vector.z = sz / s_norm;
    }
    else
    {
        // Fallback to normalized input ground_vector
        smoothed_normal_.vector.x = n_x;
        smoothed_normal_.vector.y = n_y;
        smoothed_normal_.vector.z = n_z;
    }

    smoothed_normal_.header = ground_vector.header;
    return smoothed_normal_;
}

geometry_msgs::Vector3Stamped GlobalGroundFinder::gaussian_smoothing(const geometry_msgs::Vector3Stamped &ground_vector)
{
    if (!use_gaussian_smoothing_ || !gaussian_kernel_)
    {
        return ground_vector;
    }

    double n_x = ground_vector.vector.x;
    double n_y = ground_vector.vector.y;
    double n_z = ground_vector.vector.z;
    double n_len = std::sqrt(n_x * n_x + n_y * n_y + n_z * n_z);

    if (n_len > 1e-9)
    {
        n_x /= n_len;
        n_y /= n_len;
        n_z /= n_len;
    }
    else
    {
        // Fallback to current smoothed_normal
        if (have_smoothed_normal_)
            return smoothed_normal_;
        return ground_vector;
    }

    auto result = gaussian_kernel_->filter(static_cast<float>(n_x),
                                           static_cast<float>(n_y),
                                           static_cast<float>(n_z));

    geometry_msgs::Vector3Stamped output;
    double o_x = result[0];
    double o_y = result[1];
    double o_z = result[2];
    double o_len = std::sqrt(o_x * o_x + o_y * o_y + o_z * o_z);

    if (o_len > 1e-9)
    {
        o_x /= o_len;
        o_y /= o_len;
        o_z /= o_len;
    }

    output.vector.x = o_x;
    output.vector.y = o_y;
    output.vector.z = o_z;

    smoothed_normal_ = output; // Internal state for fallback
    have_smoothed_normal_ = true;

    return output;
}

void GlobalGroundFinder::log_results(const ros::Time &stamp,
                                     const std::vector<double> &normal,
                                     double vis_score,
                                     double inlier_score,
                                     double combined_score)
{
    if (!log_file_.is_open())
    {
        return;
    }

    log_file_ << std::fixed << std::setprecision(9) << stamp.toSec() << ","
              << std::setprecision(6) << normal[0] << ","
              << normal[1] << ","
              << normal[2] << ","
              << std::setprecision(4) << vis_score << ","
              << inlier_score << ","
              << combined_score << "\n";
    log_file_.flush();
}
