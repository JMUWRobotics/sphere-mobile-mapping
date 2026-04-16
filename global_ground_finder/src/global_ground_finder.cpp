/*
 * Implementation of global map-based ground normal finder
 * Based on ground_finder.cpp by Carolin Bösch
 * Adapted for lio_sphere node, providing a global map
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
      last_pitch_(0.0),
      last_snapshot_version_(0)
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
    pnh.param<double>("smoothing_cutoff_freq", smoothing_cutoff_freq_, 2.0);
    pnh.param<double>("update_rate", update_rate_, 20.0);
    have_smoothed_normal_ = false;

    if (use_gaussian_smoothing_)
    {
        float dt = 1.0f / static_cast<float>(update_rate_);
        double sigma = (sqrt(2.0 * log(2))) / (2.0 * M_PI * smoothing_cutoff_freq_); // nominator ensures signal gain of -3dB at cutoff freq. -> see ipynb
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
            log_file_ << "timestamp,nx,ny,nz,qx,qy,qz,roll,pitch,pub_vis_score,pub_inlier_score,pub_combined_score,curr_vis_score,curr_inlier_score,curr_combined_score,inlier_count,subcloud_size,inlier_ratio,using_fallback,search_radius\n";
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

    pnh.param<std::string>("trigger_topic", trigger_topic_, "/all_pose_out"); // Estimator trigger from LIO

    pnh.param<bool>("publish_shared_map_debug", publish_shared_map_debug_, true); // DISABLED: causes double-free in multi-threaded context
    pnh.param<bool>("debug_publish", debug_publish_, false);
    pnh.param<std::string>("shared_map_debug_topic", shared_map_debug_topic_, "/global_ground_finder/shared_map_out");

    tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

    // sub_map = nh.subscribe("/map_out", 10, &GlobalGroundFinder::mapCallback, this);
    sub_trigger = nh.subscribe(trigger_topic_, 10, &GlobalGroundFinder::triggerCallback, this);
    // sub_pose = nh.subscribe("/lkf/pose", 10, &GlobalGroundFinder::poseCallback, this);

    pub_local_cloud = nh.advertise<sensor_msgs::PointCloud2>("/global_ground_finder/local_cloud", 1);
    pub_inliers = nh.advertise<sensor_msgs::PointCloud2>("/global_ground_finder/inliers", 1);
    pub_rejected_inliers = nh.advertise<sensor_msgs::PointCloud2>("/global_ground_finder/rejected_inliers", 1);
    pub_n = nh.advertise<geometry_msgs::Vector3Stamped>("/global_ground_finder/normal", 1);
    pub_scored_n = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("/global_ground_finder/scored_normal", 1);
    pub_smoothed_n = nh.advertise<geometry_msgs::Vector3Stamped>("/global_ground_finder/smoothed_normal", 1);
    pub_smoothed_scored_n = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("/global_ground_finder/smoothed_scored_normal", 1);
    pub_n_marker = nh.advertise<visualization_msgs::Marker>("/global_ground_finder/normal_marker", 1);
    pub_shared_map_debug = nh.advertise<sensor_msgs::PointCloud2>(shared_map_debug_topic_, 1);
    pub_scored_n_pandar = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("/global_ground_finder/scored_normal_pandar", 1);
    pub_smoothed_scored_n_pandar = nh.advertise<ground_finder_msgs::ScoredNormalStamped>("/global_ground_finder/smoothed_scored_normal_pandar", 1);

    initMarker();

    ROS_INFO("Global Ground Finder initialized");
    ROS_INFO("  Extraction radius: %.2f m", extraction_radius_);
    ROS_INFO("  Extraction height: %.2f m", extraction_height_);
    ROS_INFO("  Enable scoring: %s", enable_scoring_ ? "true" : "false");
    ROS_INFO("  Publish shared map debug: %s on %s",
             publish_shared_map_debug_ ? "true" : "false",
             shared_map_debug_topic_.c_str());
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
    normal_marker_.header.frame_id = "map_lio";
}

// old callback which calculated normal as soon as new registered point cloud arrives but now via memory sharing of global map
/*
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
    */

void GlobalGroundFinder::triggerCallback(const state_estimator_msgs::EstimatorConstPtr &msg)
{
    if (!msg)
    {
        return;
    }

    geometry_msgs::PoseStamped latest_pose = msg->pose;
    if (latest_pose.header.stamp.isZero())
    {
        latest_pose.header.stamp = msg->header.stamp;
    }
    if (latest_pose.header.frame_id.empty())
    {
        latest_pose.header.frame_id = msg->header.frame_id;
    }

    // Use newest pose from registered path as query pose.
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = latest_pose;
        pose_received_ = true;
    }

    if (publish_shared_map_debug_)
    {
        publishSharedMapDebug(latest_pose.header.stamp); // causes seg fault
    }

    // process on scan cadence while avoiding callback pile-up
    std::unique_lock<std::mutex> lock(processing_mutex_, std::try_to_lock);
    // ROS_INFO("owning lock: %s", lock.owns_lock() ? "true" : "false");
    if (lock.owns_lock())
    {
        processAtCurrentPose();
    }
}

void GlobalGroundFinder::publishSharedMapDebug(const ros::Time &trigger_stamp)
{

    const auto handle = lio_gf_nodelet_manager::SharedIKDTree::instance().snapshot();
    if (!handle.payload_type.empty() && handle.payload_type != typeid(pcl::PointCloud<PointType>).name())
    {
        ROS_WARN_THROTTLE(2.0, "Cannot publish shared debug map: payload type mismatch (%s)", handle.payload_type.c_str());
        return;
    }
    const auto shared_map = lio_gf_nodelet_manager::SharedIKDTree::castPayload<pcl::PointCloud<PointType>>(handle);

    if (!shared_map || shared_map->points.empty())
    {
        ROS_WARN_THROTTLE(2.0, "Cannot publish shared debug map: snapshot is missing or empty");
        return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*shared_map, map_msg);
    map_msg.header.frame_id = handle.frame_id;
    ROS_INFO("Frame: %s, publishing shared map debug with %zu points", map_msg.header.frame_id.c_str(), shared_map->points.size());
    map_msg.header.stamp = handle.stamp.isZero() ? trigger_stamp : handle.stamp;
    pub_shared_map_debug.publish(map_msg);
}

void GlobalGroundFinder::publishRejectedInliers(const pcl::PointCloud<PointType>::Ptr &rejected_cloud)
{
    if (!debug_publish_ || !rejected_cloud || rejected_cloud->points.empty())
    {
        return;
    }

    sensor_msgs::PointCloud2 rejected_msg;
    pcl::toROSMsg(*rejected_cloud, rejected_msg);
    rejected_msg.header.stamp = ros::Time::now();

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!current_pose_.header.frame_id.empty())
        {
            rejected_msg.header.frame_id = current_pose_.header.frame_id;
            if (!current_pose_.header.stamp.isZero())
            {
                rejected_msg.header.stamp = current_pose_.header.stamp;
            }
        }
        else
        {
            rejected_msg.header.frame_id = "map_lio";
        }
    }

    pub_rejected_inliers.publish(rejected_msg);
}

// cur not used and sub commented out
void GlobalGroundFinder::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Update current pose with protection
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = *msg; // pose arrives in pandar_frame currently but i need it in map_lio //TODO: check frames
        pose_received_ = true;
    }

    // Processing is triggered by scan callback so this callback remains lightweight.
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
    if (!extractLocalCloud(pose_copy, local_cloud)) // checks for map received and sufficient local points
    {
        ROS_WARN("Failed to extract local cloud");
        count_fail_++;
        return;
    }

    last_local_cloud_size_ = local_cloud->points.size();

    sensor_msgs::PointCloud2 local_msg;
    pcl::toROSMsg(*local_cloud, local_msg);
    local_msg.header.stamp = pose_copy.header.stamp;
    local_msg.header.frame_id = pose_copy.header.frame_id;
    pub_local_cloud.publish(local_msg);

    // calc ground plane
    std::vector<double> normal;
    size_t inlier_count = 0;
    pcl::PointCloud<PointType>::Ptr inlier_cloud(new pcl::PointCloud<PointType>);
    if (!fitGroundPlane(local_cloud, normal, inlier_count, inlier_cloud))
    {
        ROS_WARN("Failed to fit ground plane");
        count_fail_++;
        return;
    }

    last_inlier_count_ = inlier_count;

    sensor_msgs::PointCloud2 inliers_msg;
    pcl::toROSMsg(*inlier_cloud, inliers_msg);
    inliers_msg.header.stamp = pose_copy.header.stamp;
    inliers_msg.header.frame_id = pose_copy.header.frame_id;
    pub_inliers.publish(inliers_msg);

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
    scored_msg.header = pose_copy.header; // use pose timestamp for scored normal TOOD: check which frame. pandar_frame or map_lkf
    scored_msg.normal.x = normal[0];
    scored_msg.normal.y = normal[1];
    scored_msg.normal.z = normal[2];
    scored_msg.visibility_score = vis_score;
    scored_msg.inlier_score = inlier_score;
    scored_msg.combined_score = combined_score;

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

    if (write2file)
    {
        log_results(pose_copy.header.stamp,
                    normal,
                    pose_copy.pose,
                    scored_msg.visibility_score,
                    scored_msg.inlier_score,
                    scored_msg.combined_score,
                    vis_score,
                    inlier_score,
                    combined_score,
                    inlier_count,
                    local_cloud->points.size(),
                    using_fallback);
    }

    // Publish stuff
    geometry_msgs::Vector3Stamped n_msg;
    n_msg.header = pose_copy.header;
    n_msg.vector.x = normal[0];
    n_msg.vector.y = normal[1];
    n_msg.vector.z = normal[2];
    pub_n.publish(n_msg);
    pub_scored_n.publish(scored_msg);

    ground_finder_msgs::ScoredNormalStamped scored_msg_pandar;
    scored_msg_pandar.header.stamp = scored_msg.header.stamp;
    scored_msg_pandar.header.frame_id = "pandar_frame";
    scored_msg_pandar.visibility_score = scored_msg.visibility_score;
    scored_msg_pandar.inlier_score = scored_msg.inlier_score;
    scored_msg_pandar.combined_score = scored_msg.combined_score;

    geometry_msgs::TransformStamped t_map_lio_to_pandar;
    try
    {
        t_map_lio_to_pandar = tf_buffer.lookupTransform("pandar_frame", scored_msg.header.frame_id, ros::Time(0));
        geometry_msgs::Vector3Stamped normal_map_lio;
        normal_map_lio.header = scored_msg.header;
        normal_map_lio.vector = scored_msg.normal;

        geometry_msgs::Vector3Stamped normal_pandar;
        tf2::doTransform(normal_map_lio, normal_pandar, t_map_lio_to_pandar);
        scored_msg_pandar.normal = normal_pandar.vector; // now holds scored normal in pandar frame
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transforming scored normal to pandar_frame failed: %s", ex.what());

        // Use n from algos as fallback if transform fails -> skip scoring and sliding_window fallback
        scored_msg_pandar.normal.x = n_[0];
        scored_msg_pandar.normal.y = n_[1];
        scored_msg_pandar.normal.z = n_[2];
    }

    pub_scored_n_pandar.publish(scored_msg_pandar);

    if (enable_normal_smoothing_)
    {
        // Smooth raw normal
        geometry_msgs::Vector3Stamped smoothed_raw_n;
        if (use_gaussian_smoothing_)
        {
            smoothed_raw_n = gaussian_smoothing(n_msg);
        }
        else
        {
            smoothed_raw_n = ema_smoothing(n_msg);
        }
        smoothed_raw_n.header.stamp = n_msg.header.stamp;
        smoothed_raw_n.header.frame_id = n_msg.header.frame_id;
        pub_smoothed_n.publish(smoothed_raw_n);

        // Smoothed scored normal
        geometry_msgs::Vector3Stamped scored_n_stamped;
        scored_n_stamped.header = scored_msg.header;
        scored_n_stamped.vector = scored_msg.normal;

        // overwrite scored_n_stamped with smoothed values
        if (use_gaussian_smoothing_ == false)
        {
            scored_n_stamped = ema_smoothing(scored_n_stamped);
        }
        else
        {
            scored_n_stamped = gaussian_smoothing(scored_n_stamped);
        }

        // Wrap smoothed Vector3Stamped into ScoredNormalStamped Msg for publishing
        ground_finder_msgs::ScoredNormalStamped smoothed_scored_msg;
        smoothed_scored_msg.header = scored_n_stamped.header;
        smoothed_scored_msg.normal = scored_n_stamped.vector;
        smoothed_scored_msg.visibility_score = scored_msg.visibility_score;
        smoothed_scored_msg.inlier_score = scored_msg.inlier_score;
        smoothed_scored_msg.combined_score = scored_msg.combined_score;
        smoothed_scored_msg.header.stamp = scored_msg.header.stamp;
        smoothed_scored_msg.header.frame_id = scored_msg.header.frame_id;

        pub_smoothed_scored_n.publish(smoothed_scored_msg);

        // Transform and publish smoothed & scored normal in local pandar_frame
        ground_finder_msgs::ScoredNormalStamped smoothed_scored_msg_pandar;
        smoothed_scored_msg_pandar.header.stamp = smoothed_scored_msg.header.stamp;
        smoothed_scored_msg_pandar.header.frame_id = "pandar_frame";
        smoothed_scored_msg_pandar.visibility_score = smoothed_scored_msg.visibility_score;
        smoothed_scored_msg_pandar.inlier_score = smoothed_scored_msg.inlier_score;
        smoothed_scored_msg_pandar.combined_score = smoothed_scored_msg.combined_score;

        geometry_msgs::TransformStamped t_map_lio_to_pandar;
        try
        {
            t_map_lio_to_pandar = tf_buffer.lookupTransform("pandar_frame", smoothed_scored_msg.header.frame_id, ros::Time(0));
            geometry_msgs::Vector3Stamped normal_map_lio;
            normal_map_lio.header = smoothed_scored_msg.header;
            normal_map_lio.vector = smoothed_scored_msg.normal;

            geometry_msgs::Vector3Stamped normal_pandar;
            tf2::doTransform(normal_map_lio, normal_pandar, t_map_lio_to_pandar);
            smoothed_scored_msg_pandar.normal = normal_pandar.vector; // now holds smoothed & scored normal in pandar frame
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to transform smoothed scored normal to pandar_frame: %s", ex.what());
            // Use n from algos as fallback if transform fails -> skip scoring and sliding_window fallback
            smoothed_scored_msg_pandar.normal.x = n_[0];
            smoothed_scored_msg_pandar.normal.y = n_[1];
            smoothed_scored_msg_pandar.normal.z = n_[2];
        }
        pub_smoothed_scored_n_pandar.publish(smoothed_scored_msg_pandar);
    }

    publish_normal_marker(normal, pose_copy.header.stamp);

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
    PointType query_point; // TOOD:check in which frame i am
    query_point.x = pose.pose.position.x;
    query_point.y = pose.pose.position.y;
    query_point.z = pose.pose.position.z;

    ROS_INFO("Extracting local cloud around query point: [%.2f, %.2f, %.2f]", query_point.x, query_point.y, query_point.z);

    // Access immutable shared map snapshot and refresh local KdTree
    const auto handle = lio_gf_nodelet_manager::SharedIKDTree::instance().snapshot();
    if (!handle.payload_type.empty() && handle.payload_type != typeid(pcl::PointCloud<PointType>).name())
    {
        ROS_WARN_THROTTLE(2.0, "Shared snapshot payload type mismatch (%s)", handle.payload_type.c_str());
        return false;
    }
    const auto shared_map = lio_gf_nodelet_manager::SharedIKDTree::castPayload<pcl::PointCloud<PointType>>(handle);
    if (shared_map && !shared_map->points.empty())
    {
        if (handle.version != last_snapshot_version_)
        {
            if (handle.point_count != 0 && handle.point_count != shared_map->points.size())
            {
                ROS_WARN("SNAPCHK GF metadata mismatch: handle.point_count=%lu shared_map_size=%zu",
                         static_cast<unsigned long>(handle.point_count),
                         shared_map->points.size());
            }
            global_map_->points = shared_map->points;
            global_map_->width = shared_map->points.size();
            global_map_->height = 1;
            global_map_->is_dense = true;
            kdtree_->setInputCloud(global_map_);
            map_received_ = true;
            map_timestamp_ = handle.stamp;
            last_snapshot_version_ = handle.version;
            // ROS_INFO("Updated local KdTree from snapshot version %lu with %zu points (frame=%s, stamp=%.6f)",
            //          static_cast<unsigned long>(last_snapshot_version_),
            //          global_map_->points.size(),
            //          handle.frame_id.c_str(),
            //          handle.stamp.toSec());
        }
    }

    // Search in radius around current pose from local KdTree built from latest snapshot.
    std::vector<int> indices;
    std::vector<float> distances;

    if (map_received_ && kdtree_->radiusSearch(query_point, extraction_radius_, indices, distances) > 0)
    {
        if (!pose.header.frame_id.empty())
        {
            const auto handle_check = lio_gf_nodelet_manager::SharedIKDTree::instance().snapshot();
            if (!handle_check.frame_id.empty() && handle_check.frame_id != pose.header.frame_id)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "Frame mismatch: pose frame=%s, map frame=%s. Local cloud may be inconsistent.",
                                  pose.header.frame_id.c_str(),
                                  handle_check.frame_id.c_str());
            }
        }

        local_cloud->points.reserve(indices.size());
        for (size_t i = 0; i < indices.size(); ++i)
        {
            const auto &pt = global_map_->points[indices[i]];
            if (!pcl::isFinite(pt)) // drop invalid pts
            {
                continue;
            }
            const double height_diff = std::abs(pt.z - query_point.z); // vertical dist to curr pose's z component
            if (height_diff <= extraction_height_)                     // only keep the points within +- 0.5m vertically
            {
                local_cloud->points.push_back(pt); // add to local cloud if within height threshold
            }
        }

        local_cloud->width = local_cloud->points.size();
        local_cloud->height = 1;
        local_cloud->is_dense = true;
        return local_cloud->points.size() >= min_points_for_plane_;
    }

    // Fallback: use latest /map_out cloud when shared tree is not available.
    if (!map_received_ || global_map_->points.size() == 0)
    {
        return false;
    }

    if (kdtree_->radiusSearch(query_point, extraction_radius_, indices, distances) == 0)
    {
        return false;
    }

    // Filter by height relative to curr z
    local_cloud->points.reserve(indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const auto &pt = global_map_->points[indices[i]];
        if (!pcl::isFinite(pt)) // drop invalid pts
        {
            continue;
        }
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
                                        size_t &inlier_count,
                                        pcl::PointCloud<PointType>::Ptr &inlier_cloud)
{
    if (!local_cloud || local_cloud->points.size() < 3)
    {
        ROS_ERROR("Invalid cloud for plane fitting: nullptr=%s, size=%zu",
                  !local_cloud ? "true" : "false",
                  local_cloud ? local_cloud->points.size() : 0);
        return false;
    }

    bool success = false;
    normal.clear();
    normal.resize(3, 0.0);
    if (!inlier_cloud)
    {
        inlier_cloud.reset(new pcl::PointCloud<PointType>);
    }
    inlier_cloud->clear();

    switch (plane_algorithm_)
    {
    case PCA:
        success = fitPlanePCA(local_cloud, normal, inlier_cloud);
        inlier_count = local_cloud->points.size(); // (PCA uses all points)
        break;
    case RANSAC:
        success = fitPlaneRANSAC(local_cloud, normal, inlier_count, inlier_cloud);
        break;
    case RHT:
        success = fitPlaneRHT(local_cloud, normal, inlier_count, inlier_cloud);
        break;
    case RHT2:
        success = fitPlaneRHT2(local_cloud, normal, inlier_count, inlier_cloud);
        break;
    }

    if (!success)
    {
        ROS_WARN("Plane fitting failed with algorithm=%d", plane_algorithm_);
        return false;
    }

    // Validate normal is not NaN or zero
    if (normal.size() != 3 ||
        std::isnan(normal[0]) || std::isnan(normal[1]) || std::isnan(normal[2]) ||
        (std::abs(normal[0]) < 1e-9 && std::abs(normal[1]) < 1e-9 && std::abs(normal[2]) < 1e-9))
    {
        ROS_ERROR("Invalid normal computed: [%.6f, %.6f, %.6f]", normal[0], normal[1], normal[2]);
        return false;
    }

    // Validate it's ground (not wall)
    return validateGroundNormal(normal);
}

bool GlobalGroundFinder::fitPlanePCA(const pcl::PointCloud<PointType>::Ptr &cloud,
                                     std::vector<double> &normal,
                                     pcl::PointCloud<PointType>::Ptr &inlier_cloud)
{
    if (!cloud || cloud->points.size() < 3)
    {
        ROS_ERROR("PCA: Invalid cloud: nullptr=%s, size=%zu",
                  !cloud ? "true" : "false",
                  cloud ? cloud->points.size() : 0);
        return false;
    }

    try
    {
        pcl::PCA<PointType> pca;
        pca.setInputCloud(cloud);
        Eigen::Matrix3f eigen_vecs = pca.getEigenVectors();

        // Validate eigen vectors are valid (not NaN, not infinite)
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (std::isnan(eigen_vecs(i, j)) || std::isinf(eigen_vecs(i, j)))
                {
                    ROS_ERROR("PCA: Invalid eigen vector at [%d,%d]: %f", i, j, eigen_vecs(i, j));
                    return false;
                }
            }
        }

        // Normal is smallest eigenvector (3rd column)
        normal.clear();
        normal.resize(3);
        normal[0] = eigen_vecs(0, 2);
        normal[1] = eigen_vecs(1, 2);
        normal[2] = eigen_vecs(2, 2);

        if (!inlier_cloud)
        {
            inlier_cloud.reset(new pcl::PointCloud<PointType>);
        }
        *inlier_cloud = *cloud;

        ROS_DEBUG("PCA: Computed normal [%.6f, %.6f, %.6f]", normal[0], normal[1], normal[2]);
        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("PCA failed with exception: %s", e.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR("PCA failed with unknown exception");
        return false;
    }
}

bool GlobalGroundFinder::fitPlaneRANSAC(const pcl::PointCloud<PointType>::Ptr &cloud,
                                        std::vector<double> &normal,
                                        size_t &inlier_count,
                                        pcl::PointCloud<PointType>::Ptr &inlier_cloud)
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
            const bool valid_ground = validateGroundNormal(test_normal);
            if (valid_ground)
            {
                normal = test_normal;
                if (!inlier_cloud)
                {
                    inlier_cloud.reset(new pcl::PointCloud<PointType>);
                }
                inlier_cloud->clear();
                inlier_cloud->points.reserve(inliers.size());
                for (const int idx : inliers)
                {
                    if (idx >= 0 && static_cast<size_t>(idx) < cloud_work->points.size())
                    {
                        inlier_cloud->points.push_back(cloud_work->points[idx]);
                    }
                }
                inlier_cloud->width = inlier_cloud->points.size();
                inlier_cloud->height = 1;
                inlier_cloud->is_dense = true;
                return true;
            }
            else if (is_last)
            {
                return false;
            }

            pcl::PointCloud<PointType>::Ptr rejected_cloud(new pcl::PointCloud<PointType>);
            rejected_cloud->points.reserve(inliers.size());
            for (const int idx : inliers)
            {
                if (idx >= 0 && static_cast<size_t>(idx) < cloud_work->points.size())
                {
                    rejected_cloud->points.push_back(cloud_work->points[idx]);
                }
            }
            rejected_cloud->width = rejected_cloud->points.size();
            rejected_cloud->height = 1;
            rejected_cloud->is_dense = true;
            publishRejectedInliers(rejected_cloud);

            // Remove inliers and try again (wall removal)
            pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
            cloud_filtered->points.reserve(cloud_work->points.size() - inliers.size());

            std::set<int> inlier_set;
            inlier_set.clear();
            for (const int idx : inliers)
            {
                if (idx >= 0 && static_cast<size_t>(idx) < cloud_work->points.size())
                {
                    inlier_set.insert(idx);
                }
            }

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
            cloud_work->is_dense = true;

            if (cloud_work->points.size() < 3)
            {
                return false;
            }
        }
        catch (...)
        {
            ROS_ERROR("exception in fitPlaneRANSAC");
            return false;
        }
    }

    return false;
}

bool GlobalGroundFinder::fitPlaneRHT(const pcl::PointCloud<PointType>::Ptr &cloud,
                                     std::vector<double> &normal,
                                     size_t &inlier_count,
                                     pcl::PointCloud<PointType>::Ptr &inlier_cloud)
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
                if (!inlier_cloud)
                {
                    inlier_cloud.reset(new pcl::PointCloud<PointType>);
                }
                inlier_cloud->clear();
                for (size_t i = 0; i < cloud_work->points.size(); ++i)
                {
                    const PointType &p = cloud_work->points[i];
                    double distance = std::abs(p.x * normal[0] + p.y * normal[1] + p.z * normal[2] - rho);
                    if (distance <= 0.05) // 5cm threshold
                    {
                        inlier_count++;
                        inlier_cloud->points.push_back(p);
                    }
                }
                inlier_cloud->width = inlier_cloud->points.size();
                inlier_cloud->height = 1;
                inlier_cloud->is_dense = true;
                return true;
            }
            else if (last_iteration)
            {
                return false;
            }

            pcl::PointCloud<PointType>::Ptr rejected_cloud(new pcl::PointCloud<PointType>);

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
                else
                {
                    rejected_cloud->points.push_back(p);
                }
            }

            rejected_cloud->width = rejected_cloud->points.size();
            rejected_cloud->height = 1;
            rejected_cloud->is_dense = true;
            publishRejectedInliers(rejected_cloud);

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
                                      size_t &inlier_count,
                                      pcl::PointCloud<PointType>::Ptr &inlier_cloud)
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
                        if (!inlier_cloud)
                        {
                            inlier_cloud.reset(new pcl::PointCloud<PointType>);
                        }
                        inlier_cloud->clear();
                        inlier_cloud->points.reserve(inliers.size());
                        for (const int idx : inliers)
                        {
                            if (idx >= 0 && static_cast<size_t>(idx) < cloud_work->points.size())
                            {
                                inlier_cloud->points.push_back(cloud_work->points[idx]);
                            }
                        }
                        inlier_cloud->width = inlier_cloud->points.size();
                        inlier_cloud->height = 1;
                        inlier_cloud->is_dense = true;
                        return true;
                    }
                    else if (last_iteration)
                    {
                        return false;
                    }

                    pcl::PointCloud<PointType>::Ptr rejected_cloud(new pcl::PointCloud<PointType>);
                    rejected_cloud->points.reserve(inliers.size());
                    for (const int idx : inliers)
                    {
                        if (idx >= 0 && static_cast<size_t>(idx) < cloud_work->points.size())
                        {
                            rejected_cloud->points.push_back(cloud_work->points[idx]);
                        }
                    }
                    rejected_cloud->width = rejected_cloud->points.size();
                    rejected_cloud->height = 1;
                    rejected_cloud->is_dense = true;
                    publishRejectedInliers(rejected_cloud);

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
    //  Validate normal vector before processing
    if (normal.size() != 3)
    {
        ROS_ERROR("Normal vector has wrong size: %zu (expected 3)", normal.size());
        return false;
    }

    // Check for NaN or infinity
    for (int i = 0; i < 3; ++i)
    {
        if (std::isnan(normal[i]) || std::isinf(normal[i]))
        {
            ROS_ERROR("Normal contains invalid value at index %d: %f", i, normal[i]);
            return false;
        }
    }

    // Check if vector is nearly zero
    double norm_sq = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
    if (norm_sq < 1e-12) // Avoid division by near-zero
    {
        ROS_ERROR("Normal vector is nearly zero: norm_sq=%.2e", norm_sq);
        return false;
    }

    // Now safe to normalize
    normalize_vector(normal);

    // Check against down vector [0, 0, -1]
    std::vector<double> down = {0.0, 0.0, -1.0};
    double dot = dot_product(normal, down); // TODO: make sure we are in the correct frame here for "normal"

    // Check if it's a wall (perpendicular to down)
    if (std::abs(dot) < wall_threshold_)
    {
        ROS_DEBUG("Rejecting plane: too parallel to horizontal (dot=%.3f, threshold=%.3f)", dot, wall_threshold_);
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
    if (!current_pose_.header.frame_id.empty())
        normal_marker_.header.frame_id = current_pose_.header.frame_id;
    else
        normal_marker_.header.frame_id = "map_lio";

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

    pub_n_marker.publish(normal_marker_);
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
                                     const geometry_msgs::Pose &query_pose,
                                     double pub_vis_score,
                                     double pub_inlier_score,
                                     double pub_combined_score,
                                     double curr_vis_score,
                                     double curr_inlier_score,
                                     double curr_combined_score,
                                     size_t inlier_count,
                                     size_t subcloud_size,
                                     bool using_fallback)
{
    if (!log_file_.is_open())
    {
        return;
    }

    const double inlier_ratio = (subcloud_size > 0)
                                    ? static_cast<double>(inlier_count) / static_cast<double>(subcloud_size)
                                    : 0.0;
    const double roll_deg = last_roll_ * 180.0 / M_PI;
    const double pitch_deg = last_pitch_ * 180.0 / M_PI;

    log_file_ << std::fixed << std::setprecision(9) << stamp.toSec() << ","
              << std::setprecision(6) << normal[0] << ","
              << normal[1] << ","
              << normal[2] << ","
              << query_pose.position.x << ","
              << query_pose.position.y << ","
              << query_pose.position.z << ","
              << roll_deg << ","
              << pitch_deg << ","
              << std::setprecision(4) << pub_vis_score << ","
              << pub_inlier_score << ","
              << pub_combined_score << ","
              << curr_vis_score << ","
              << curr_inlier_score << ","
              << curr_combined_score << ","
              << inlier_count << ","
              << subcloud_size << ","
              << std::setprecision(6) << inlier_ratio << ","
              << (using_fallback ? 1 : 0) << ","
              << std::setprecision(4) << extraction_radius_ << "\n";
    log_file_.flush();
}
