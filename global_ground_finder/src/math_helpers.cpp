#include "math_helpers.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/surface/convex_hull.h>
#include <limits>

double deg2rad(const double deg)
{
    return deg * M_PI / 180.0;
}

void subtract_points(const PointType &p1, const PointType &p2, std::vector<double> &d)
{
    d[0] = p1.x - p2.x;
    d[1] = p1.y - p2.y;
    d[2] = p1.z - p2.z;
}

double dot_product(const std::vector<double> &v1, const std::vector<double> &v2)
{
    if (v1.size() != v2.size())
    {
        return 0.0; // Size mismatch
    }
    double product = 0.0;
    for (size_t i = 0; i < v1.size(); i++)
        product += v1[i] * v2[i];
    return product;
}

void cross_product(const std::vector<double> &v1, const std::vector<double> &v2, std::vector<double> &res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void normalize_vector(std::vector<double> &v)
{
    if (v.size() < 3)
    {
        return; // Invalid size - exit silently
    }

    double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    if (norm < 1e-12) // Avoid division by zero
    {
        // Set to default (pointing down)
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = -1.0;
        return;
    }

    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
}

std::vector<double> convert2polar(std::vector<double> cartesian_vector)
{
    double phi, theta, theta0;
    std::vector<double> polar = {0.0, 0.0, 0.0};

    normalize_vector(cartesian_vector);
    phi = acos(cartesian_vector[2]);

    if (fabs(phi) < 0.0001)
    {
        theta = 0.0;
    }
    else if (fabs(M_PI - phi) < 0.0001)
    {
        theta = 0.0;
    }
    else
    {
        if (fabs(cartesian_vector[0] / sin(phi)) > 1.0)
        {
            if (cartesian_vector[0] / sin(phi) < 0)
            {
                theta0 = M_PI;
            }
            else
            {
                theta0 = 0.0;
            }
        }
        else
        {
            theta0 = acos(cartesian_vector[0] / sin(phi));
        }

        double sintheta = cartesian_vector[1] / sin(phi);
        double EPS = 0.0001;

        if (fabs(sin(theta0) - sintheta) < EPS)
        {
            theta = theta0;
        }
        else if (fabs(sin(2 * M_PI - theta0) - sintheta) < EPS)
        {
            theta = 2 * M_PI - theta0;
        }
        else
        {
            theta = 0;
            printf("ERROR! Error converting cartesian coordinates into polar\n");
        }
    }

    polar[0] = phi;
    polar[1] = theta;
    polar[2] = -1.0;
    return polar;
}

bool validatePointDistributionFromEigenvalues(float lambda1, float lambda2, float lambda3,
                                              double eigenvalue_ratio_threshold,
                                              double &eigenvalue_ratio)
{
    // Avoid division by zero or negative eigenvalues
    if (lambda1 < 1e-9 || lambda2 < 1e-9 || lambda3 < 1e-9)
    {
        eigenvalue_ratio = 1.0; // Invalid -> reject
        return false;
    }

    // Ground plane: 2 large eigenvalues (XY), 1 small (Z)
    // Wall: more uniform eigenvalue distribution
    // Ratio = smallest / sum of two largest
    // Ground should have ratio << 1, wall ratio ~= large value
    double sum_large = static_cast<double>(lambda1) + static_cast<double>(lambda2);
    eigenvalue_ratio = static_cast<double>(lambda3) / sum_large;

    // Lower ratio = more planar (good for ground)
    // If ratio exceeds threshold, likely a wall or non-planar structure
    if (eigenvalue_ratio > eigenvalue_ratio_threshold)
    {
        return false; // Rejected: too non-planar
    }

    return true; // Accepted: looks like planar ground
}

bool validateZMeanDeviation(const pcl::PointCloud<PointType>::Ptr &cloud,
                            double robot_z,
                            double max_z_deviation,
                            double &z_mean)
{
    if (!cloud || cloud->points.empty())
    {
        z_mean = robot_z; // Default fallback
        return false;
    }

    // Compute mean Z and min/max Z
    double z_sum = 0.0;
    double z_min = std::numeric_limits<double>::max();
    double z_max = -std::numeric_limits<double>::max();
    size_t valid_count = 0;

    for (const auto &pt : cloud->points)
    {
        if (pcl::isFinite(pt))
        {
            z_sum += pt.z;
            z_min = std::min(z_min, static_cast<double>(pt.z));
            z_max = std::max(z_max, static_cast<double>(pt.z));
            valid_count++;
        }
    }

    if (valid_count == 0)
    {
        z_mean = robot_z; // Default fallback
        return false;
    }

    z_mean = z_sum / static_cast<double>(valid_count);

    // Check 1: Deviation of mean from robot Z
    double mean_deviation = std::abs(z_mean - robot_z);

    // Check 2: Z-spread (vertical extent) - walls will have large spread
    double z_spread = z_max - z_min;
    double max_z_spread = max_z_deviation * 2.0; // Allow spread up to 2x the deviation threshold

    ROS_INFO("validateZMeanDeviation: z_mean=%.3f, robot_z=%.3f, mean_dev=%.3f m, z_spread=%.3f m (thresholds: %.3f, %.3f)",
             z_mean, robot_z, mean_deviation, z_spread, max_z_deviation, max_z_spread);

    if (mean_deviation > max_z_deviation)
    {
        ROS_INFO("  REJECTED: mean deviation %.3f > threshold %.3f", mean_deviation, max_z_deviation);
        return false; // Rejected: mean Z too far from robot
    }

    if (z_spread > max_z_spread)
    {
        ROS_INFO("  REJECTED: z_spread %.3f > threshold %.3f (indicates wall/vertical surface)", z_spread, max_z_spread);
        return false; // Rejected: points span too much vertically (wall indicator)
    }

    return true; // Accepted: reasonable Z distribution
}

bool computeConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                             geometry_msgs::Point &hull_center)
{
    if (!cloud || cloud->points.empty())
    {
        ROS_WARN("computeConvexHullCenter: Invalid cloud");
        return false;
    }

    // Compute centroid of inlier cloud directly (not hull vertices)
    // This gives the true center of mass of all points on the plane
    double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
    double x_min = std::numeric_limits<double>::max();
    double x_max = -std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double y_max = -std::numeric_limits<double>::max();
    double z_min = std::numeric_limits<double>::max();
    double z_max = -std::numeric_limits<double>::max();
    size_t valid_count = 0;

    for (const auto &pt : cloud->points)
    {
        if (pcl::isFinite(pt))
        {
            x_sum += pt.x;
            y_sum += pt.y;
            z_sum += pt.z;

            x_min = std::min(x_min, static_cast<double>(pt.x));
            x_max = std::max(x_max, static_cast<double>(pt.x));
            y_min = std::min(y_min, static_cast<double>(pt.y));
            y_max = std::max(y_max, static_cast<double>(pt.y));
            z_min = std::min(z_min, static_cast<double>(pt.z));
            z_max = std::max(z_max, static_cast<double>(pt.z));

            valid_count++;
        }
    }

    if (valid_count == 0)
    {
        ROS_WARN("computeConvexHullCenter: No valid points in cloud");
        return false;
    }

    hull_center.x = x_sum / static_cast<double>(valid_count);
    hull_center.y = y_sum / static_cast<double>(valid_count);
    hull_center.z = z_sum / static_cast<double>(valid_count);

    ROS_INFO("computeConvexHullCenter: Cloud size=%zu, valid=%zu",
             cloud->points.size(), valid_count);
    ROS_INFO("  Bounds: X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f]",
             x_min, x_max, y_min, y_max, z_min, z_max);
    ROS_INFO("  Centroid: [%.3f, %.3f, %.3f]",
             hull_center.x, hull_center.y, hull_center.z);
    return true;
}

bool validateConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                              const geometry_msgs::Point &robot_pose,
                              double max_hull_distance,
                              double &hull_distance,
                              geometry_msgs::Point &hull_center)
{
    if (!cloud || cloud->points.empty())
    {
        hull_distance = std::numeric_limits<double>::max();
        hull_center.x = 0.0;
        hull_center.y = 0.0;
        hull_center.z = 0.0;
        ROS_WARN("validateConvexHullCenter: Invalid inlier cloud");
        return false;
    }

    geometry_msgs::Point local_hull_center;
    if (!computeConvexHullCenter(cloud, local_hull_center))
    {
        hull_distance = std::numeric_limits<double>::max();
        hull_center.x = 0.0;
        hull_center.y = 0.0;
        hull_center.z = 0.0;
        ROS_WARN("validateConvexHullCenter: Failed to compute hull center");
        return false;
    }

    hull_center = local_hull_center;
    // Compute 3D Euclidean distance from robot to hull center
    double dx = hull_center.x - robot_pose.x;
    double dy = hull_center.y - robot_pose.y;
    double dz = hull_center.z - robot_pose.z;

    hull_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    ROS_INFO("validateConvexHullCenter: robot at [%.3f, %.3f, %.3f]",
             robot_pose.x, robot_pose.y, robot_pose.z);
    ROS_INFO("  centroid at [%.3f, %.3f, %.3f]",
             hull_center.x, hull_center.y, hull_center.z);
    ROS_INFO("  delta: dx=%.3f, dy=%.3f, dz=%.3f",
             dx, dy, dz);
    ROS_INFO("  distance=%.3f m (threshold=%.3f m)",
             hull_distance, max_hull_distance);

    // Check if hull center is within max distance thresh
    if (hull_distance > max_hull_distance)
    {
        ROS_INFO("  REJECTED: distance %.3f > threshold %.3f", hull_distance, max_hull_distance);
        return false; // hull center too far from robot
    }

    ROS_INFO("  ACCEPTED: distance %.3f <= threshold %.3f", hull_distance, max_hull_distance);
    return true;
}
