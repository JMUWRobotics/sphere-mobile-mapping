#include "ground_finder.h"
#include <ground_finder_msgs/ScoredNormalStamped.h>
#include <iomanip>

void GroundFinder::initMarker()
{
    n_marker.type = visualization_msgs::Marker::ARROW;
    n_marker.action = visualization_msgs::Marker::ADD;
    n_marker.pose.orientation.x = 0.0;
    n_marker.pose.orientation.y = 0.0;
    n_marker.pose.orientation.z = 0.0;
    n_marker.pose.orientation.w = 1.0;
    n_marker.scale.x = 0.02;
    n_marker.scale.y = 0.06;
    n_marker.scale.z = 0.06;
    n_marker.color.a = 1.0;
    n_marker.color.r = 1.0;
    n_marker.color.g = 0.0;
    n_marker.color.b = 0.0;
}

int64_t GroundFinder::delete_points(pcl::PointCloud<PointType>::Ptr &cur_scan, pcl::PointIndices::Ptr &indices, const bool negativ)
{
    pcl::ExtractIndices<PointType> extract;
    auto start_del = std::chrono::high_resolution_clock::now();
    extract.setInputCloud(cur_scan);
    extract.setIndices(indices);
    extract.setNegative(negativ);
    extract.filter(*cur_scan);
    auto end_del = std::chrono::high_resolution_clock::now();
    auto duration_del = std::chrono::duration_cast<std::chrono::microseconds>(end_del - start_del).count();
    if (!quiet)
        ROS_INFO("[GF] Delete points from cloud takes: %0.3f ms", float(duration_del) / 1e3);

    return duration_del;
}

int64_t GroundFinder::downsample(pcl::PointCloud<PointType>::Ptr &cur_scan)
{
    auto start_ds = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(cur_scan);
    ds.setLeafSize(ds_size, ds_size, ds_size);
    ds.filter(*cur_scan);
    auto end_ds = std::chrono::high_resolution_clock::now();
    auto duration_ds = std::chrono::duration_cast<std::chrono::microseconds>(end_ds - start_ds).count();
    if (!quiet)
        ROS_INFO("[GF] Downsampling cloud takes: %0.6f ms", float(duration_ds) / 1e3);

    // Write to file
    if (write2file)
        csv << duration_ds << ",";

    return duration_ds;
}

int64_t GroundFinder::filter_cloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan)
{
    pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
    del_points->indices.resize(cur_scan->points.size());

    // Helper variables
    int count_dp = 0;
    double d_sqr = 0.0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < (*cur_scan).size(); i++)
    {
        PointType p_i = cur_scan->points[i];
        // Check if point lies within radius to be filtered out
        d_sqr = p_i.x * p_i.x + p_i.y * p_i.y + p_i.z * p_i.z;
        if (d_sqr <= radius_filter * radius_filter)
        {
            del_points->indices[count_dp++] = i;
        }
    }
    del_points->indices.resize(count_dp);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if (!quiet)
        ROS_INFO("[GF] Looping through cloud takes: %0.6f ms", float(duration_loop) / 1e3);

    // Delete points from cloud
    auto duration_del = delete_points(cur_scan, del_points, true);

    // Write to file
    if (write2file)
        csv << "0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::filter_cloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan)
{
    // Initialize + Build balanced kd-tree from point cloud
    pcl::KdTreeFLANN<PointType> kd_tree_del;
    auto start_build = std::chrono::high_resolution_clock::now();
    kd_tree_del.setInputCloud(cur_scan);
    auto end_build = std::chrono::high_resolution_clock::now();
    auto duration_build = std::chrono::duration_cast<std::chrono::microseconds>(end_build - start_build).count();
    if (!quiet)
        ROS_INFO("[GF] Building tree takes: %0.3f ms", float(duration_build) / 1e3);

    // Radius search
    PointType origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    std::vector<int> search_radius_result;
    std::vector<float> search_radius_distances;
    auto start_search = std::chrono::high_resolution_clock::now();
    kd_tree_del.radiusSearch(origin, radius_filter, search_radius_result, search_radius_distances);
    auto end_search = std::chrono::high_resolution_clock::now();
    auto duration_search = std::chrono::duration_cast<std::chrono::microseconds>(end_search - start_search).count();
    if (!quiet)
        ROS_INFO("[GF] Radius search takes: %0.3f ms", float(duration_search) / 1e3);

    // Delete points from cloud
    pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
    del_points->indices = search_radius_result;
    auto duration_del = delete_points(cur_scan, del_points, true);

    // Write to file
    if (write2file)
        csv << duration_build << "," << duration_search << "," << duration_del << ",";

    return duration_build + duration_search + duration_del;
}

int64_t GroundFinder::create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point)
{
    pcl::PointIndices::Ptr points(new pcl::PointIndices());
    pcl::PointIndices::Ptr points_backup(new pcl::PointIndices());
    points->indices.resize(cur_scan->points.size());
    points_backup->indices.resize(cur_scan->points.size());

    // Helper variables
    std::vector<double> d = {0.0, 0.0, 0.0};
    double d_sqr = 0.0;
    int count_p = 0;
    int count_pb = 0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < (*cur_scan).size(); i++)
    {
        PointType p_i = cur_scan->points[i];
        // distance squared to query point
        subtract_points(p_i, query_point, d);
        d_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        // Check if point lies within sphere around the query point
        if (d_sqr <= radius_subcloud * radius_subcloud)
        {
            points_backup->indices[count_pb++] = i;
            // Now check if also point lies within cheese form (= sphere around query point with height limit)
            if (fabs(dot_product(d, n)) <= height_subcloud * 0.5)
            {
                points->indices[count_p++] = i;
            }
        }
    }
    points->indices.resize(count_p);
    points_backup->indices.resize(count_pb);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if (!quiet)
        ROS_INFO("[GF] Looping through cloud takes: %0.6f ms", float(duration_loop) / 1e3);

    // Delete points from cloud
    int64_t duration_del;
    if (points->indices.size() < 3)
    {
        // Not enough points found due to bad cheese form or bad n vector -> take whole sphere
        duration_del = delete_points(cur_scan, points_backup);
    }
    else
    {
        duration_del = delete_points(cur_scan, points);
    }

    // Write to file
    if (write2file)
        csv << "0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::create_subcloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan, int k, PointType query_point)
{
    // Build balanced kd-tree from (filtered) point cloud
    pcl::KdTreeFLANN<PointType> kd_tree;
    auto start_build = std::chrono::high_resolution_clock::now();
    kd_tree.setInputCloud(cur_scan);
    auto end_build = std::chrono::high_resolution_clock::now();
    auto duration_build = std::chrono::duration_cast<std::chrono::microseconds>(end_build - start_build).count();
    if (!quiet)
        ROS_INFO("[GF] Building tree takes: %0.6f ms", float(duration_build) / 1e3);

    // KNN search
    std::vector<int> search_result;
    std::vector<float> search_distances;
    auto start_search = std::chrono::high_resolution_clock::now();
    kd_tree.nearestKSearch(query_point, k, search_result, search_distances); // NOTE: not ranged search!
    auto end_search = std::chrono::high_resolution_clock::now();
    auto duration_search = std::chrono::duration_cast<std::chrono::microseconds>(end_search - start_search).count();
    if (!quiet)
        ROS_INFO("[GF] Search kNN takes: %0.6f ms", float(duration_search) / 1e3);

    // Delete Points from cloud
    pcl::PointIndices::Ptr sub_points(new pcl::PointIndices());
    sub_points->indices = search_result;
    auto duration_del = delete_points(cur_scan, sub_points);

    // Write to file
    if (write2file)
        csv << duration_build << "," << duration_search << "," << duration_del << ",";

    return duration_build + duration_search + duration_del;
}

int64_t GroundFinder::filter_create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point)
{
    pcl::PointIndices::Ptr points(new pcl::PointIndices());
    pcl::PointIndices::Ptr points_backup(new pcl::PointIndices());
    points->indices.resize(cur_scan->points.size());
    points_backup->indices.resize(cur_scan->points.size());

    // Helper variables
    std::vector<double> d = {0.0, 0.0, 0.0};
    double d1_sqr = 0.0, d2_sqr = 0.0;
    int count_p = 0;
    int count_pb = 0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < (*cur_scan).size(); i++)
    {
        PointType p_i = cur_scan->points[i];
        // distance squared to center (0,0,0)
        d1_sqr = p_i.x * p_i.x + p_i.y * p_i.y + p_i.z * p_i.z;
        // distance squared to query point
        subtract_points(p_i, query_point, d);
        d2_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        // Check if point lies outside of filter radius and within sphere around the query RHT
        if (d1_sqr > radius_filter * radius_filter && d2_sqr <= radius_subcloud * radius_subcloud)
        {
            points_backup->indices[count_pb++] = i;
            // Now check if also point lies within cheese form (= sphere around query point with height limit)
            if (fabs(dot_product(d, n)) <= height_subcloud * 0.5)
            {
                points->indices[count_p++] = i;
            }
        }
    }
    points->indices.resize(count_p);
    points_backup->indices.resize(count_pb);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if (!quiet)
        ROS_INFO("[GF] Looping through cloud takes: %0.6f ms", float(duration_loop) / 1e3);

    // Delete points from cloud
    int64_t duration_del;
    if (points->indices.size() < 3)
    {
        // Not enough points found due to bad cheese form or bad n vector -> take whole sphere
        duration_del = delete_points(cur_scan, points_backup);
    }
    else
    {
        duration_del = delete_points(cur_scan, points);
    }

    // Write to file
    if (write2file)
        csv << "0,0,0,0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::determine_n_ground_plane(pcl::PointCloud<PointType>::Ptr &cur_scan, PlaneSegm type, geometry_msgs::Vector3Stamped &n_msg)
{
    int64_t duration_plane = 0.0;
    // Select Plane algorithm
    switch (type)
    {
    case LSF:
    {
        float curv;
        Eigen::Vector4f params;
        auto start_plane = std::chrono::high_resolution_clock::now();
        pcl::computePointNormal(*cur_scan, params, curv); // NOTE: eigenvector = already normalized :)
        // Set new normal vector
        n[0] = params[0];
        n[1] = params[1];
        n[2] = params[2];
        pcl::PointCloud<PointType>::Ptr validation_cloud(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cur_scan, *validation_cloud);
        if (!convert_n_to_map_frame(n_msg))
            return -1000;

        std::vector<double> validation_normal = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
        double robot_z = last_lio_pose ? last_lio_pose->pose.pose.position.z : 0.0;
        if (!validateGroundNormal(validation_normal, validation_cloud, robot_z))
            return -1000;
        n_msg.vector.x = validation_normal[0];
        n_msg.vector.y = validation_normal[1];
        n_msg.vector.z = validation_normal[2];

        // publish points used for normal computation
        sensor_msgs::PointCloud2 sub_cloud_msg;
        pcl::PointCloud<PointType>::Ptr final(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cur_scan, *final);
        sub_cloud_msg.header.stamp = n_msg.header.stamp;
        sub_cloud_msg.header.frame_id = "pandar_frame";
        pub_inliers.publish(sub_cloud_msg);

        auto end_plane = std::chrono::high_resolution_clock::now();
        duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
        break;
    }
    case PCA:
    {
        pcl::PCA<PointType> pca;
        auto start_plane = std::chrono::high_resolution_clock::now();
        pca.setInputCloud(cur_scan);
        Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
        Eigen::Vector3f eigen_vals = pca.getEigenValues();
        // Set new normal vector
        n[0] = eigen_vecs.coeff(0, 2);
        n[1] = eigen_vecs.coeff(1, 2);
        n[2] = eigen_vecs.coeff(2, 2);
        pcl::PointCloud<PointType>::Ptr validation_cloud(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cur_scan, *validation_cloud);
        if (!convert_n_to_map_frame(n_msg))
            return -1000;

        std::vector<double> validation_normal = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
        double robot_z = last_lio_pose ? last_lio_pose->pose.pose.position.z : 0.0;
        if (!validateGroundNormal(validation_normal, validation_cloud, robot_z,
                                  eigen_vals(0), eigen_vals(1), eigen_vals(2),
                                  eigen_vecs(2, 0), eigen_vecs(2, 1)))
            return -1000;
        n_msg.vector.x = validation_normal[0];
        n_msg.vector.y = validation_normal[1];
        n_msg.vector.z = validation_normal[2];

        // publish points used for normal computation
        sensor_msgs::PointCloud2 sub_cloud_msg;
        pcl::PointCloud<PointType>::Ptr final(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cur_scan, *final);
        pcl::toROSMsg(*final, sub_cloud_msg);
        sub_cloud_msg.header.stamp = n_msg.header.stamp;
        sub_cloud_msg.header.frame_id = "pandar_frame";
        pub_inliers.publish(sub_cloud_msg);

        auto end_plane = std::chrono::high_resolution_clock::now();
        duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
        break;
    }
    case RANSAC:
    {
        // NOTE: unter 0.01 kaum besseres ergebnis dafür langsamer, über 0.02 deutlich ungenauer und erst wirklich schneller ab 0.05 (sehr ungenau schon)
        double dist_thresh = 0.01;
        std::vector<int> inliers;
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
        pcl::PointCloud<PointType>::Ptr accepted_inliers(new pcl::PointCloud<PointType>);

        auto start_plane = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < max_iterations_plane_detection; i++)
        {
            // Created RandomSampleConsensus object the appropriate model
            pcl::SampleConsensusModelPlane<PointType>::Ptr model(new pcl::SampleConsensusModelPlane<PointType>(cur_scan));
            pcl::RandomSampleConsensus<PointType> ransac(model);
            ransac.setDistanceThreshold(dist_thresh);
            // NOTE: max_iterations_ = def: 10000, but = attribute from parent class; not used in ransac! = no effect just propability!
            // NOTE: Propability (def:0.99) - basically no effect on speed and insignificant one on accuracy (worse at propability < 0.5)!
            // PCA for normal vector calculation
            pcl::PCA<PointType> pca;

            try
            {
                ransac.computeModel();
                ransac.getInliers(inliers);

                // /* LSF */
                // pcl::copyPointCloud(*cur_scan, inliers, *tmp);
                // pcl::computePointNormal(*tmp, params, curv); // NOTE: eigenvector = already normalized :)
                // Set new normal vector
                // n[0] = params[0];
                // n[1] = params[1];
                // n[2] = params[2];

                /* PCA */
                inliers_ptr->indices = inliers;
                accepted_inliers.reset(new pcl::PointCloud<PointType>);
                pcl::copyPointCloud(*cur_scan, inliers_ptr->indices, *accepted_inliers);
            }
            catch (pcl::InitFailedException)
            {
                return -1000;
            }

            bool last_iteration = (i == max_iterations_plane_detection - 1);

            if (!accepted_inliers || accepted_inliers->points.size() < 3)
            {
                if (last_iteration)
                    return -1000;

                delete_points(cur_scan, inliers_ptr, true);
                inliers = {};
                inliers_ptr->indices = {};
                if (cur_scan->size() < 3)
                    return -1000;
                continue;
            }

            pca.setInputCloud(accepted_inliers);
            Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
            Eigen::Vector3f eigen_vals = pca.getEigenValues();

            // Set new normal vector
            n[0] = eigen_vecs.coeff(0, 2);
            n[1] = eigen_vecs.coeff(1, 2);
            n[2] = eigen_vecs.coeff(2, 2);

            // Check if ground plane found
            if (!convert_n_to_map_frame(n_msg, last_iteration))
            {
                if (last_iteration)
                    return -1000;
                break;
            }

            std::vector<double> validation_normal = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
            double robot_z = last_lio_pose ? last_lio_pose->pose.pose.position.z : 0.0;
            if (!validateGroundNormal(validation_normal, accepted_inliers, robot_z,
                                      eigen_vals(0), eigen_vals(1), eigen_vals(2),
                                      eigen_vecs(2, 0), eigen_vecs(2, 1)))
            {
                if (last_iteration)
                    return -1000;

                delete_points(cur_scan, inliers_ptr, true);
                inliers = {};
                inliers_ptr->indices = {};
                if (cur_scan->size() < 3)
                    return -1000;
                continue;
            }

            n_msg.vector.x = validation_normal[0];
            n_msg.vector.y = validation_normal[1];
            n_msg.vector.z = validation_normal[2];

            last_inlier_count = accepted_inliers ? accepted_inliers->size() : 0;
            break;
        }

        auto end_plane = std::chrono::high_resolution_clock::now();
        duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();

        if (accepted_inliers && !accepted_inliers->points.empty())
        {
            sensor_msgs::PointCloud2 sub_cloud_msg;
            pcl::toROSMsg(*accepted_inliers, sub_cloud_msg);
            sub_cloud_msg.header.stamp = n_msg.header.stamp;
            sub_cloud_msg.header.frame_id = "pandar_frame";
            pub_inliers.publish(sub_cloud_msg);
        }

        break;
    }
    case RHT:
    {
        // Accumulator (90 and 180 = 2° accuracy)
        // int rhoNum = 7, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
        int rhoNum = radius_sphere + 1, phiNum = 180, thetaNum = 360, rhoMax = 1500, accumulatorMax = 20;
        //  NOTE: rhoMax = max distance from plane to origin of frame (in our case = radius sphere + little wiggle room with int = 1)
        //  NOTE: rhoNum = no effect on accuracy but makes it faster if low! now = 7 -> roughly speed of ran  -- increase for finer plane distance resolution -- distance of plane from origin grid
        //  NOTE: phiNum and thetaNum define accuracy of plane orientation! but higher = slower! -- angular resolution for normal direction grid
        //  NOTE: accumulatorMax = Maximal accumulator count; Any cell in accumulator > accumulatorMax = potential plane candidate.
        Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
        // Hough object
        // double minDist = 0;                                  // min distance between points [cm]
        // double maxDist = std::numeric_limits<double>::max(); // max distance between points [cm]
        double minDist = 1.5;
        double maxDist = 50.0;

        /* approach for tuning:  */

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // storing inliers for visualization
        inliers->indices.resize(cur_scan->points.size());
        pcl::PointCloud<PointType>::Ptr accepted_inliers(new pcl::PointCloud<PointType>);

        auto start_plane = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < max_iterations_plane_detection; i++)
        {
            Hough hough(cur_scan, &acc, minDist, maxDist);
            double rho = hough.RHT(n); // NOTE: eigenvector = already normalized :)

            bool last_iteration = (i == max_iterations_plane_detection - 1);
            // Transform normal into map frame before validation
            if (!convert_n_to_map_frame(n_msg, last_iteration))
                return -1000;

            // Check if try failed
            if (rho != -1)
            {
                // If not remove all points from cloud and re-do RHT alg
                pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
                del_points->indices.resize(cur_scan->points.size());
                int del_count = 0;
                int inlier_count = 0;

                for (int i = 0; i < (*cur_scan).size(); i++)
                {
                    PointType p_i = cur_scan->points[i];
                    double distance = fabs(p_i.x * n[0] + p_i.y * n[1] + p_i.z * n[2] - rho);
                    // Check if point lies on wrongly detected plane
                    if (distance <= 0.05)
                    {
                        del_points->indices[del_count++] = i;
                    }
                    else
                    {
                        inliers->indices[inlier_count++] = i;
                    }
                }
                inliers->indices.resize(inlier_count);
                del_points->indices.resize(del_count);

                accepted_inliers.reset(new pcl::PointCloud<PointType>);
                pcl::copyPointCloud(*cur_scan, inliers->indices, *accepted_inliers);

                pcl::PCA<PointType> pca;
                pca.setInputCloud(accepted_inliers);
                Eigen::Matrix3f eigen_vecs = pca.getEigenVectors();
                Eigen::Vector3f eigen_vals = pca.getEigenValues();

                if (!convert_n_to_map_frame(n_msg, last_iteration))
                {
                    return -1000;
                }

                std::vector<double> validation_normal = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
                double robot_z = last_lio_pose ? last_lio_pose->pose.pose.position.z : 0.0;
                if (!validateGroundNormal(validation_normal, accepted_inliers, robot_z,
                                          eigen_vals(0), eigen_vals(1), eigen_vals(2),
                                          eigen_vecs(2, 0), eigen_vecs(2, 1)))
                {
                    if (last_iteration)
                        return -1000;

                    delete_points(cur_scan, inliers, true);
                    if (cur_scan->size() < 3)
                        return -1000;
                    continue;
                }

                n_msg.vector.x = validation_normal[0];
                n_msg.vector.y = validation_normal[1];
                n_msg.vector.z = validation_normal[2];

                last_inlier_count = accepted_inliers->size();

                // publish points used for normal computation (should be inliers here)
                sensor_msgs::PointCloud2 sub_cloud_msg;
                pcl::toROSMsg(*accepted_inliers, sub_cloud_msg);
                sub_cloud_msg.header.stamp = n_msg.header.stamp;
                sub_cloud_msg.header.frame_id = "pandar_frame";
                pub_inliers.publish(sub_cloud_msg);

                break;

                // TODO comment out!
                // Publish new subcloud for next try
                // sensor_msgs::PointCloud2 sub_cloud_msg_2;
                // pcl::toROSMsg(*cur_scan, sub_cloud_msg_2);
                // sub_cloud_msg_2.header.stamp = n_msg.header.stamp;
                // sub_cloud_msg_2.header.frame_id = "pandar_frame";
                // pub_test2.publish(sub_cloud_msg_2);
            }
        }
        auto end_plane = std::chrono::high_resolution_clock::now();
        duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
        break;
    }
    case RHT2: // RHT followed by PCA -- evaluated to be least recommended but has much more good ground points
    {
        // Accumulator (36 and 72 = 5° accuracy)
        // int rhoNum = 7, phiNum = 36, thetaNum = 72, rhoMax = 5, accumulatorMax = 10;
        int rhoNum = radius_sphere + 1, phiNum = 180, thetaNum = 360, rhoMax = 1500, accumulatorMax = 2;
        Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
        // Hough object
        // double minDist = 0;                                  // min distance between points
        // double maxDist = std::numeric_limits<double>::max(); // max distance between points
        double minDist = 1.5;
        double maxDist = 50.0;

        std::vector<double> temp_n = {0.0, 0.0, 0.0};

        auto start_plane = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < max_iterations_plane_detection; i++)
        {
            bool last_iteration = (i == max_iterations_plane_detection - 1);

            Hough hough(cur_scan, &acc, minDist, maxDist);
            double rho = hough.RHT(temp_n); // NOTE: eigenvector = already normalized :)

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            inliers->indices.resize(cur_scan->points.size());
            int count = 0;
            pcl::PointCloud<PointType>::Ptr accepted_inliers(new pcl::PointCloud<PointType>);
            Eigen::Matrix3f eigen_vecs;
            Eigen::Vector3f eigen_vals;

            try
            {
                // Fit plane if a plane was found (rho != 1)
                if (rho != -1)
                {
                    // Determine inliers of found rough plane
                    for (int i = 0; i < (*cur_scan).size(); i++)
                    {
                        PointType p_i = cur_scan->points[i];
                        double distance = fabs(p_i.x * n[0] + p_i.y * n[1] + p_i.z * n[2] - rho);
                        // Check if point lies on detected plane
                        if (distance <= 0.05)
                        {
                            inliers->indices[count++] = i;
                        }
                    }
                    inliers->indices.resize(count);
                    // Check if enough points for PCA if not just retry RHT
                    if (!quiet)
                        ROS_WARN("[GF] Inliers num: %ld", inliers->indices.size());
                    if (!last_iteration && count < 3)
                        continue;

                    // PCA for normal vector calculation
                    pcl::PCA<PointType> pca;
                    pca.setInputCloud(cur_scan);
                    pca.setIndices(inliers);
                    eigen_vecs = pca.getEigenVectors();
                    eigen_vals = pca.getEigenValues();

                    accepted_inliers.reset(new pcl::PointCloud<PointType>);
                    pcl::copyPointCloud(*cur_scan, inliers->indices, *accepted_inliers);

                    last_inlier_count = inliers->indices.size(); // TODO: double check

                    // Set new normal vector
                    n[0] = eigen_vecs.coeff(0, 2);
                    n[1] = eigen_vecs.coeff(1, 2);
                    n[2] = eigen_vecs.coeff(2, 2);
                }
            }
            catch (pcl::InitFailedException)
            {
                ROS_ERROR("[GF] Not enough points for PCA!");
                convert_n_to_map_frame(n_msg, true);
                return -1000;
            }

            // Transform normal into map frame before validation
            if (!convert_n_to_map_frame(n_msg, last_iteration))
                return -1000;

            std::vector<double> validation_normal = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
            double robot_z = last_lio_pose ? last_lio_pose->pose.pose.position.z : 0.0;
            if (rho != -1)
            {
                if (!validateGroundNormal(validation_normal, accepted_inliers, robot_z,
                                          eigen_vals(0), eigen_vals(1), eigen_vals(2),
                                          eigen_vecs(2, 0), eigen_vecs(2, 1)))
                {
                    if (last_iteration)
                        return -1000;

                    delete_points(cur_scan, inliers, true);
                    if (cur_scan->size() < 3)
                    {
                        ROS_ERROR("[GF] Not enough points after delete in cur_scan! num_points < 3");
                        convert_n_to_map_frame(n_msg, true);
                        return -1000;
                    }
                    continue;
                }

                n_msg.vector.x = validation_normal[0];
                n_msg.vector.y = validation_normal[1];
                n_msg.vector.z = validation_normal[2];

                sensor_msgs::PointCloud2 sub_cloud_msg;
                pcl::toROSMsg(*accepted_inliers, sub_cloud_msg);
                sub_cloud_msg.header.stamp = n_msg.header.stamp;
                sub_cloud_msg.header.frame_id = "pandar_frame";
                pub_inliers.publish(sub_cloud_msg);

                break;
            }

            // If not remove those points from cloud and re-do RHT alg
            if (rho != -1)
            {
                delete_points(cur_scan, inliers, true);
                // If not enough points left to fit plane = stop plane detection
                if (cur_scan->size() < 3)
                {
                    ROS_ERROR("[GF] Not enough points after delete in cur_scan! num_points < 3");
                    convert_n_to_map_frame(n_msg, true);
                    return -1000;
                }

                // TODO comment out!
                // Publish (filtered) subcloud
                //     sensor_msgs::PointCloud2 sub_cloud_msg;
                //     pcl::toROSMsg(*cur_scan, sub_cloud_msg);
                //     sub_cloud_msg.header.stamp = n_msg.header.stamp;
                //     sub_cloud_msg.header.frame_id = "pandar_frame";
                //     pub_test2.publish(sub_cloud_msg);
            }
        }

        auto end_plane = std::chrono::high_resolution_clock::now();
        duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
        break;
    }
    }

    if (!quiet)
    {
        ROS_INFO("[GF] Plane Segmentation takes: %0.6f ms", float(duration_plane) / 1e3);
        // ROS_INFO("Normal Vector: nx: %.3f, ny: %.3f, nz: %.3f", n[0], n[1], n[2]);
    }
    return duration_plane;
}

bool GroundFinder::convert_n_to_map_frame(geometry_msgs::Vector3Stamped &n_msg, const bool &last_iteration)
{
    (void)last_iteration;

    geometry_msgs::TransformStamped t;
    try
    {
        // Listen to tf tree for transformation
        t = tf_buffer.lookupTransform("map_lio", "pandar_frame", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("[GF] Failed to listen to tf tree!\n\n");
        return false;
    }
    // Create Vector n in pandar frame
    geometry_msgs::Vector3Stamped n_pandar;
    n_pandar.vector.x = n[0];
    n_pandar.vector.y = n[1];
    n_pandar.vector.z = n[2];

    tf2::doTransform(n_pandar, n_msg, t);

    // Make sure it always points into ground
    std::vector<double> down = {0.0, 0.0, -1.0};
    std::vector<double> n_map_lio = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
    double dot_prod = dot_product(down, n_map_lio);

    if (!quiet)
    {
        double inclination = std::acos(std::max(-1.0, std::min(1.0, dot_prod))) * 180.0 / M_PI;
        ROS_INFO("[GF][frame] pandar->map_lio normal=(%.5f, %.5f, %.5f) dot_down=%.5f inclination=%.5f deg",
                 n_msg.vector.x, n_msg.vector.y, n_msg.vector.z, dot_prod, inclination);
    }

    if (enable_plane_angle_validation && std::abs(dot_prod) < wall_threshold)
    {
        if (!quiet)
        {
            ROS_WARN("[GF][angle] rejected as wall: |dot_down|=%.5f < wall_threshold=%.5f",
                     std::abs(dot_prod), wall_threshold);
        }

        if (last_iteration && subcloud == GEOMETRICAL)
        {
            try
            {
                t = tf_buffer.lookupTransform("pandar_frame", "map_lio", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("[GF] Failed to listen to tf tree!\n\n");
                return false;
            }

            geometry_msgs::Vector3Stamped n_map;
            n_map.vector.x = 0.0;
            n_map.vector.y = 0.0;
            n_map.vector.z = -1.0;

            tf2::doTransform(n_map, n_pandar, t);
            n[0] = n_pandar.vector.x;
            n[1] = n_pandar.vector.y;
            n[2] = n_pandar.vector.z;

            if (!quiet)
                ROS_ERROR("[GF] Resetting n!");
        }

        return false;
    }

    // Flip normal to point downward if needed
    if (dot_prod < 0)
    {
        n[0] = -n[0];
        n[1] = -n[1];
        n[2] = -n[2];
        n_msg.vector.x = -n_msg.vector.x;
        n_msg.vector.y = -n_msg.vector.y;
        n_msg.vector.z = -n_msg.vector.z;
    }

    if (!quiet)
    {
        n_map_lio = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
        dot_prod = dot_product(down, n_map_lio);
        double inclination = acos(dot_prod) * 180 / M_PI;
        ROS_INFO("[GF] Inclination of the plane: %.5f", inclination);
    }

    return true;
}

bool GroundFinder::validateGroundNormal(std::vector<double> &normal,
                                        const pcl::PointCloud<PointType>::Ptr &inlier_cloud,
                                        double robot_z,
                                        float lambda1,
                                        float lambda2,
                                        float lambda3,
                                        float v1_z,
                                        float v2_z)
{
    if (normal.size() != 3)
    {
        return false;
    }

    for (double value : normal)
    {
        if (std::isnan(value) || std::isinf(value))
        {
            return false;
        }
    }

    double norm_sq = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
    if (norm_sq < 1e-12)
    {
        return false;
    }

    normalize_vector(normal);

    std::vector<double> down = {0.0, 0.0, -1.0};
    double dot = dot_product(normal, down);

    if (enable_eigenvalue_validation && lambda1 > 0.0f && lambda2 > 0.0f && lambda3 > 0.0f &&
        v1_z != 0.0f && v2_z != 0.0f)
    {
        double eigenvalue_ratio = 0.0;
        bool eigenvalue_valid = validatePointDistributionFromEigenvalues(lambda1, lambda2, lambda3,
                                                                         eigenvalue_ratio_threshold,
                                                                         eigenvalue_ratio);
        float max_dominant_z = std::max(std::abs(v1_z), std::abs(v2_z));
        bool eigenvector_valid = max_dominant_z < max_eigenvector_z_component;

        if (!quiet)
        {
            ROS_INFO("[GF][validate][eigen] lambda=(%.5f, %.5f, %.5f) ratio=%.5f threshold=%.5f dominant_z=%.5f threshold=%.5f result=%s",
                     lambda1, lambda2, lambda3,
                     eigenvalue_ratio,
                     eigenvalue_ratio_threshold,
                     max_dominant_z,
                     max_eigenvector_z_component,
                     (eigenvalue_valid && eigenvector_valid) ? "pass" : "fail");
        }

        if (!eigenvalue_valid || !eigenvector_valid)
        {
            if (!quiet)
            {
                ROS_WARN("[GF][validate][eigen] rejected: eigenvalue_valid=%s eigenvector_valid=%s",
                         eigenvalue_valid ? "true" : "false",
                         eigenvector_valid ? "true" : "false");
            }
            return false;
        }
    }

    if (enable_z_mean_validation && inlier_cloud && !inlier_cloud->points.empty())
    {
        double z_mean = 0.0;
        if (!validateZMeanDeviation(inlier_cloud, robot_z, max_z_deviation, z_mean))
        {
            if (!quiet)
            {
                ROS_WARN("[GF][validate][z_mean] rejected: z_mean=%.5f robot_z=%.5f max_z_deviation=%.5f",
                         z_mean, robot_z, max_z_deviation);
            }
            return false;
        }

        if (!quiet)
        {
            ROS_INFO("[GF][validate][z_mean] pass: z_mean=%.5f robot_z=%.5f max_z_deviation=%.5f",
                     z_mean, robot_z, max_z_deviation);
        }
    }

    if (enable_convex_hull_validation && inlier_cloud && !inlier_cloud->points.empty())
    {
        geometry_msgs::Point hull_center;
        double hull_distance = 0.0;
        geometry_msgs::Point robot_pose;
        robot_pose.x = 0.0;
        robot_pose.y = 0.0;
        robot_pose.z = robot_z;

        if (!validateConvexHullCenter(inlier_cloud, robot_pose, max_hull_distance, hull_distance, hull_center))
        {
            if (!quiet)
            {
                ROS_WARN("[GF][validate][hull] rejected: hull_center=(%.5f, %.5f, %.5f) hull_distance=%.5f max_hull_distance=%.5f",
                         hull_center.x, hull_center.y, hull_center.z,
                         hull_distance, max_hull_distance);
            }
            return false;
        }

        if (!quiet)
        {
            ROS_INFO("[GF][validate][hull] pass: hull_center=(%.5f, %.5f, %.5f) hull_distance=%.5f max_hull_distance=%.5f",
                     hull_center.x, hull_center.y, hull_center.z,
                     hull_distance, max_hull_distance);
        }
    }

    if (!quiet)
    {
        ROS_INFO("[GF][validate] accepted plane");
    }

    return true;
}

void GroundFinder::scan_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!quiet)
    {
        ROS_WARN("[GF] entered scan callback");
    }
    // Convert PointCloud2 (sensor msg) to PointCloud (pcl)
    pcl::PointCloud<PointType>::Ptr cur_scan(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cur_scan);

    auto start_total = std::chrono::high_resolution_clock::now();
    // Total duration [ns]
    int64_t duration_total = 0;

    // Calculation of query point for creation of subcloud (used for both approaches)
    PointType query_point = {0.0, 0.0, 0.0};
    query_point.x = n[0] * radius_sphere;
    query_point.y = n[1] * radius_sphere;
    query_point.z = n[2] * radius_sphere;

    // ---------------------- Preproccessing of scan ----------------------

    if (filtering == GEOMETRICAL && subcloud == GEOMETRICAL)
    {
        // If both approaches = geo, then only looping once through entire point cloud (faster)
        duration_total += downsample(cur_scan);
        duration_total += filter_create_subcloud_geo(cur_scan, query_point);
    }
    else
    {
        // A) Filter cloud: Take out reflections from sphere + hands & downsample or not (filtering == NONE)
        int64_t duration_filter = 0;
        if (filtering == GEOMETRICAL)
        {
            duration_filter += downsample(cur_scan);
            duration_filter += filter_cloud_geo(cur_scan);
        }
        else if (filtering == KD_TREE)
        {
            duration_filter += downsample(cur_scan);
            duration_filter += filter_cloud_kdt(cur_scan);
        }
        else
        {
            if (write2file)
                csv << "0,0,0,0,";
        }

        // B) Create subcloud
        int64_t duration_subcloud = 0;
        if (subcloud == GEOMETRICAL)
        {
            duration_subcloud = create_subcloud_geo(cur_scan, query_point);
        }
        else if (subcloud == KD_TREE)
        {
            duration_subcloud = create_subcloud_kdt(cur_scan, k, query_point);
        }
        // Total preprocessing time
        duration_total = duration_filter + duration_subcloud;
    }
    if (!quiet)
        ROS_WARN("[GF] Total preprocessing time: %0.6f ms", float(duration_total) / 1e3);
    if (write2file)
        csv << duration_total << ",";

    // Publish (filtered) subcloud
    sensor_msgs::PointCloud2 sub_cloud_msg;
    pcl::toROSMsg(*cur_scan, sub_cloud_msg);
    sub_cloud_msg.header.stamp = msg->header.stamp;
    sub_cloud_msg.header.frame_id = msg->header.frame_id;
    pub_subcloud.publish(sub_cloud_msg); // evtl pointer auf subcloud für inliers mit gutem score merken um mehrere zu größerer inlier cloud zu bauen und davon normalenvektor

    last_subcloud_size = sub_cloud_msg.data.size(); // TODO: double check

    // ---------------------- Plane segmentation ----------------------
    const int64_t duration_preprocessing = duration_total;

    geometry_msgs::Vector3Stamped n_msg;
    n_msg.header.frame_id = "map_lio";
    n_msg.header.stamp = msg->header.stamp;
    // Determine normal vector of ground in map_lio frame incl. ensuring it represents ground and points into ground
    auto duration_plane = determine_n_ground_plane(cur_scan, plane_alg, n_msg);

    if (timing_csv_enabled_)
    {
        const bool plane_success = duration_plane >= 0;
        int64_t duration_total_timing = duration_preprocessing;
        if (plane_success)
        {
            duration_total_timing += duration_plane;
        }

        std::lock_guard<std::mutex> lock(timing_csv_mutex_);
        if (timing_csv_file_.is_open())
        {
            timing_csv_file_ << std::fixed << std::setprecision(6)
                             << msg->header.stamp.toSec() << ","
                             << (static_cast<double>(duration_preprocessing) / 1000.0) << ","
                             << (static_cast<double>(duration_plane) / 1000.0) << ","
                             << (static_cast<double>(duration_total_timing) / 1000.0) << ","
                             << (plane_success ? 1 : 0) << "\n";
            timing_csv_file_.flush();
        }
    }

    // Write to file
    if (write2file)
        csv << duration_plane << ",";

    if (duration_plane < 0)
    {
        ROS_ERROR("[GF] Plane segmentation fault!\n");
        count_fail++;
        // Write to file
        if (write2file)
            csv << "-1000,-1,-1,-1," << plane_counter << "\n";
        return;
    }

    // ---------------------- Plane Score Computation & Sliding Window ----------------------

    ground_finder_msgs::ScoredNormalStamped scored_msg;
    scored_msg.header = n_msg.header;
    scored_msg.normal = n_msg.vector;

    bool using_fallback = false;
    bool fallback_unavailable = false;
    double vis_score = 1.0;
    double inlier_score = 1.0;
    double combined_score = 1.0;

    if (enable_scoring)
    {
        // compute curr score
        auto [vis_score_computed, inlier_score_computed] = compute_plane_scores(last_lio_pose, last_inlier_count, last_subcloud_size);
        vis_score = vis_score_computed;
        inlier_score = inlier_score_computed;
        combined_score = combine_scores(vis_score, inlier_score);

        scored_msg.visibility_score = vis_score;
        scored_msg.inlier_score = inlier_score;
        scored_msg.combined_score = combined_score;
        scored_normals_sliding_window.push_back(scored_msg);

        if (scored_normals_sliding_window.size() > MAX_WINDOW_SIZE)
        {
            scored_normals_sliding_window.pop_front();
        }

        // ROS_INFO_THROTTLE(1.0, "[GF] Current normal score: %.3f (vis=%.3f, inlier=%.3f) | inliers=%zu/%zu", combined_score, vis_score, inlier_score, last_inlier_count, last_subcloud_size);
    }
    else
    {
        // Scoring disabled, use default score of 1.0
        scored_msg.visibility_score = 1.0;
        scored_msg.inlier_score = 1.0;
        scored_msg.combined_score = 1.0;
    }

    // ---------------------- Fallback Selection from Sliding Window ----------------------
    geometry_msgs::Vector3Stamped final_n = n_msg;
    double final_score = combined_score;

    if (enable_scoring && combined_score < score_threshold)
    {
        if (!quiet)
        {
            ROS_WARN("[GF] Current score (%.3f) below threshold (%.3f), searching history...",
                     combined_score, score_threshold);
        }

        // Find best candidate in sliding window using max_element with custom comparator (lambda function)
        auto fallback = std::max_element(scored_normals_sliding_window.begin(),
                                         scored_normals_sliding_window.end(),
                                         [](const ground_finder_msgs::ScoredNormalStamped &a,
                                            const ground_finder_msgs::ScoredNormalStamped &b)
                                         {
                                             return a.combined_score < b.combined_score;
                                         });
        if (fallback != scored_normals_sliding_window.end() && fallback->combined_score >= min_score_sliding_window)
        {
            final_n.vector.x = fallback->normal.x;
            final_n.vector.y = fallback->normal.y;
            final_n.vector.z = fallback->normal.z;
            final_score = fallback->combined_score;
            using_fallback = true;

            // Update scored_msg with fallback's scores and normal
            scored_msg.normal = fallback->normal;
            scored_msg.visibility_score = fallback->visibility_score;
            scored_msg.inlier_score = fallback->inlier_score;
            scored_msg.combined_score = fallback->combined_score;

            if (!quiet)
            {
                ROS_WARN("[GF] Using best historical normal (score=%.5f, age=%.1f ms)",
                         final_score,
                         (msg->header.stamp - fallback->header.stamp).toNSec() / 1e6);
            }
        }
        else
        {
            if (!quiet)
            {
                ROS_WARN("[GF] No suitable normal in sliding window found (min_score=%.3f)", min_score_sliding_window);
            }
            fallback_unavailable = true;
        }
    }

    // ---------------------- Timing & Publishing ----------------------

    // Save current scores (before they )might be overwritten by fallback)
    double curr_vis_score = vis_score;
    double curr_inlier_score = inlier_score;
    double curr_combined_score = combined_score;

    double inlier_ratio = (last_subcloud_size > 0) ? static_cast<double>(last_inlier_count) / static_cast<double>(last_subcloud_size) : 0.0;
    scored_normals_log << std::fixed << std::setprecision(6) << msg->header.stamp.toSec() << ","
                       << std::defaultfloat
                       << scored_msg.normal.x << ","
                       << scored_msg.normal.y << ","
                       << scored_msg.normal.z << ","
                       << (last_roll * 180.0 / M_PI) << ","
                       << (last_pitch * 180.0 / M_PI) << ","
                       << scored_msg.visibility_score << "," // could be overwritten by fallback msg
                       << scored_msg.inlier_score << ","     // could be overwritten by fallback msg
                       << scored_msg.combined_score << ","   // could be overwritten by fallback msg
                       << curr_vis_score << ","
                       << curr_inlier_score << ","
                       << curr_combined_score << ","
                       << last_inlier_count << ","
                       << last_subcloud_size << ","
                       << inlier_ratio << ","
                       << (using_fallback ? 1 : 0) << ","
                       << (fallback_unavailable ? 1 : 0) << "\n"; // 1: fallback tried but no good candidate in window
    scored_normals_log.flush();

    /*
    Raw Normal Vector
    */

    // Publish raw normal vector
    pub_n.publish(n_msg);

    /*
    Scored Normal Vector
    */
    // Publish scored normal (contains current or fallback scores)
    pub_scored_n.publish(scored_msg);

    // Transform and Publish scored normal in local pandar_frame

    ground_finder_msgs::ScoredNormalStamped scored_msg_pandar;
    scored_msg_pandar.header.stamp = scored_msg.header.stamp;
    scored_msg_pandar.header.frame_id = "pandar_frame";
    scored_msg_pandar.visibility_score = scored_msg.visibility_score;
    scored_msg_pandar.inlier_score = scored_msg.inlier_score;
    scored_msg_pandar.combined_score = scored_msg.combined_score;

    geometry_msgs::TransformStamped t_map_lio_to_pandar;
    try
    {
        t_map_lio_to_pandar = tf_buffer.lookupTransform("pandar_frame", "map_lio", ros::Time(0));
        geometry_msgs::Vector3Stamped normal_map_lio;
        normal_map_lio.header = scored_msg.header;
        normal_map_lio.vector = scored_msg.normal;

        geometry_msgs::Vector3Stamped normal_pandar;
        tf2::doTransform(normal_map_lio, normal_pandar, t_map_lio_to_pandar);
        scored_msg_pandar.normal = normal_pandar.vector; // now holds scored normal in pandar frame
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("[GF] Transforming scored normal to pandar_frame failed: %s", ex.what());

        // Use n from algos as fallback if transform fails -> skip scoring and sliding_window fallback
        scored_msg_pandar.normal.x = n[0];
        scored_msg_pandar.normal.y = n[1];
        scored_msg_pandar.normal.z = n[2];
    }

    pub_scored_n_pandar.publish(scored_msg_pandar);

    if (!quiet)
    {
        if (!enable_scoring)
        {
            ROS_INFO("[GF] Published scored normal: SCORING DISABLED (score=1.0)");
        }
        else if (using_fallback)
        {
            ROS_INFO("[GF] Published scored normal [map_lio]: FALLBACK (combined=%.3f, vis=%.3f, inlier=%.3f)",
                     scored_msg.combined_score, scored_msg.visibility_score, scored_msg.inlier_score);
        }
        else
        {
            ROS_INFO("[GF] Published scored normal [map_lio]: CURRENT (combined=%.3f, vis=%.3f, inlier=%.3f)",
                     scored_msg.combined_score, scored_msg.visibility_score, scored_msg.inlier_score);
        }
    }

    // Publish smoothed normal vectors (same timestamp)
    if (enable_normal_smoothing)
    {
        // Smooth raw normal
        geometry_msgs::Vector3Stamped smoothed_raw_n;
        if (use_gaussian_smoothing == false)
        {
            smoothed_raw_n = ema_smoothing(n_msg);
        }
        else
        {
            smoothed_raw_n = gaussian_smoothing(n_msg);
        }

        smoothed_raw_n.header.stamp = n_msg.header.stamp;
        smoothed_raw_n.header.frame_id = n_msg.header.frame_id;
        pub_smoothed_n.publish(smoothed_raw_n);

        // Smooth scored normal (current or fallback)
        geometry_msgs::Vector3Stamped scored_n_stamped;
        scored_n_stamped.header = scored_msg.header;
        scored_n_stamped.vector = scored_msg.normal;

        // overwrite scored_n_stamped with smoothed values
        if (use_gaussian_smoothing == false)
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
            t_map_lio_to_pandar = tf_buffer.lookupTransform("pandar_frame", "map_lio", ros::Time(0));
            geometry_msgs::Vector3Stamped normal_map_lio;
            normal_map_lio.header = smoothed_scored_msg.header;
            normal_map_lio.vector = smoothed_scored_msg.normal;

            geometry_msgs::Vector3Stamped normal_pandar;
            tf2::doTransform(normal_map_lio, normal_pandar, t_map_lio_to_pandar);
            smoothed_scored_msg_pandar.normal = normal_pandar.vector; // now holds smoothed & scored normal in pandar frame
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("[GF] Failed to transform smoothed scored normal to pandar_frame: %s", ex.what());
            // Use n from algos as fallback if transform fails -> skip scoring and sliding_window fallback
            smoothed_scored_msg_pandar.normal.x = n[0];
            smoothed_scored_msg_pandar.normal.y = n[1];
            smoothed_scored_msg_pandar.normal.z = n[2];
        }
        pub_smoothed_scored_n_pandar.publish(smoothed_scored_msg_pandar);
    }

    // Update normal vector n_marker
    n_marker.header.frame_id = msg->header.frame_id;
    n_marker.header.stamp = msg->header.stamp;
    geometry_msgs::Point start, end;
    start.x = query_point.x;
    start.y = query_point.y;
    start.z = query_point.z;
    end.x = (start.x + n[0]) / 3.0;
    end.y = (start.y + n[1]) / 3.0;
    end.z = (start.z + n[2]) / 3.0;
    n_marker.points = {start, end};
    // Publish n_marker
    pub_vis_n.publish(n_marker);

    // Total time
    duration_total += duration_plane;
    if (!quiet)
        ROS_WARN("[GF] Total time: %0.6f ms; fails: %d\n", float(duration_total) / 1e3, count_fail);
    // Write to file
    if (write2file)
        csv << duration_total << "," << n_msg.vector.x << "," << n_msg.vector.y << "," << n_msg.vector.z << ",";
    auto end_total = std::chrono::high_resolution_clock::now();
    ROS_WARN("[GF] Total time: %0.6f ms; fails: %d\n",
             std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count() / 1000.0, count_fail);
    // Plane counter
    if (write2file)
        csv << plane_counter << "\n";

    auto total_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count();
    if (!quiet)
        ROS_WARN("[GF] Total time: %0.6f ms; fails: %d\n", float(total_time_us) / 1e3, count_fail);
    if (write2file)
        csv << total_time_us << "," << n_msg.vector.x << "," << n_msg.vector.y << "," << n_msg.vector.z << "," << plane_counter << "\n";
}

void GroundFinder::scan_callback_count(const std_msgs::EmptyConstPtr &msg)
{
    if (!quiet)
    {
        ROS_WARN("[GF] MSG on 'plane' received -> Indicatiing new plane! Increasing counter.");
    }
    plane_counter++;
}

void GroundFinder::lio_pose_callback(const state_estimator_msgs::EstimatorConstPtr &msg)
{
    last_lio_pose = msg;

    if (!enable_view_score)
        enable_view_score = true;
}

// ---------- Ground Vector Smoothing ----------
geometry_msgs::Vector3Stamped GroundFinder::ema_smoothing(const geometry_msgs::Vector3Stamped &ground_vector)
{
    // pass-through if disabled
    if (!enable_normal_smoothing)
    {
        return ground_vector;
    }

    geometry_msgs::Vector3Stamped n = ground_vector;
    double n_x = n.vector.x;
    double n_y = n.vector.y;
    double n_z = n.vector.z;
    double n_norm = std::sqrt(n_x * n_x + n_z * n_z + n_y * n_y);

    if (n_norm > 1e-9)
    {
        // normalize input vector
        n_x /= n_norm;
        n_y /= n_norm;
        n_z /= n_norm;
    }
    else
    {
        // incoming vector invalid -> return existing smoothed normal if present
        if (have_smoothed_normal)
            return smoothed_normal;
        return ground_vector;
    }

    // for first call: init smoothed_normal
    if (!have_smoothed_normal)
    {
        smoothed_normal.vector.x = n_x;
        smoothed_normal.vector.y = n_y;
        smoothed_normal.vector.z = n_z;
        have_smoothed_normal = true;
        return smoothed_normal;
    }

    // exponential moving average
    double a = normal_smoothing_alpha;    // smoothing factor [0,1] from rosparam
    double px = smoothed_normal.vector.x; // previous x
    double py = smoothed_normal.vector.y; // previous y
    double pz = smoothed_normal.vector.z; // previous z

    double sx = a * n_x + (1.0 - a) * px; // smoothed x
    double sy = a * n_y + (1.0 - a) * py; // smoothed y
    double sz = a * n_z + (1.0 - a) * pz; // smoothed z

    double s_norm = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (s_norm > 1e-9) // smoothed normal is valid
    {
        smoothed_normal.vector.x = sx / s_norm;
        smoothed_normal.vector.y = sy / s_norm;
        smoothed_normal.vector.z = sz / s_norm;
    }
    else
    {
        // fallback to normalized input ground_vector
        smoothed_normal.vector.x = n_x;
        smoothed_normal.vector.y = n_y;
        smoothed_normal.vector.z = n_z;
    }

    smoothed_normal.header = ground_vector.header;
    return smoothed_normal;
}

geometry_msgs::Vector3Stamped GroundFinder::gaussian_smoothing(const geometry_msgs::Vector3Stamped &ground_vector)
{
    if (use_gaussian_smoothing == false)
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
        // fallback to curr smoothed_normal if present
        if (have_smoothed_normal)
            return smoothed_normal;
        return ground_vector;
    }
    auto result = gaussian_kernel->filter(static_cast<float>(n_x),
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

    smoothed_normal = output; // internal state for fallback
    have_smoothed_normal = true;
    return output;
}

// ---------- Score computation ----------------
// returns <visibility_score, inlier_normalized>
std::pair<double, double> GroundFinder::compute_plane_scores(const state_estimator_msgs::EstimatorConstPtr &msg, size_t inliers_count, size_t subcloud_size)
{

    // --------------visibility_score----------------
    // visibility fallback to 1.0 (full vis) if no KF pose available
    double visibility_score = 1.0;
    if (enable_view_score == false || !msg.get())
    {
        ROS_WARN_THROTTLE(2.0, "[GF] No lio pose received, using default visibility_score=1.0 (full visibility)");
    }
    else
    {
        // ----
        // Robot Pose in local (pandar) frame used for score
        // ----

        //  Get transformation from map_lio to pandar_frame
        geometry_msgs::TransformStamped t_map_lio_to_pandar;
        try
        {
            t_map_lio_to_pandar = tf_buffer.lookupTransform("pandar_frame", "map_lio", ros::Time(0)); // (target_frame, src_frame,...)
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("[GF] Failed to get transform from map_lio to pandar_frame: %s", ex.what());
            return std::make_pair(1.0, 0.0);
        }

        // Debug: Extract original angles in map_lio frame
        tf2::Quaternion q_temp;
        tf2::fromMsg(t_map_lio_to_pandar.transform.rotation, q_temp);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(q_temp).getRPY(roll, pitch, yaw);

        last_roll = roll;
        last_pitch = pitch;

        double roll_optim = std::abs(std::sin(2.0 * roll)); // Min at (0 and roll=+-90 deg), Max at (roll=+-45 deg)
        double roll_score = 0.1 + 0.9 * roll_optim;         // [0.1...1.0] to avoid zero visibility

        double pitch_optim = std::abs(std::sin(2.0 * pitch)); // Min at (0 and pitch=+-90 deg), Max at (pitch=+-45 deg)
        double pitch_score = 0.1 + 0.9 * pitch_optim;         // [0.1...1.0] to avoid zero visibility

        visibility_score = roll_score * pitch_score;
        last_visibility_score = visibility_score;
    }

    // --------------inlier_normalized----------------
    double inlier_ratio = 0.0;
    if (subcloud_size > 0)
        inlier_ratio = static_cast<double>(inliers_count) / static_cast<double>(subcloud_size);

    double inlier_normalized = 0.0;
    if (inliers_count >= min_inliers && inlier_ratio > 0.0)
        inlier_normalized = std::min(std::max(inlier_ratio / inlier_scale, 0.0), 1.0); // clamp to [0,1] -> 1.0 if min inlier_scale reached

    // ROS_INFO_THROTTLE(5.0, "[GF] Plane Scores -- visibility_score: %.5f, inlier_ratio: %.5f, inlier_normalized: %.5f (inliers=%zu, min_inliers=%zu, inlier_scale=%.3f)",
    //                   visibility_score, inlier_ratio, inlier_normalized, inliers_count, min_inliers, inlier_scale);
    return std::make_pair(visibility_score, inlier_normalized);
}

double GroundFinder::combine_scores(double visibility_score, double inlier_score)
{
    // Weighted sum (TODO: finetune weights)
    return weight_visibility * visibility_score + weight_inlier_ratio * inlier_score;
}