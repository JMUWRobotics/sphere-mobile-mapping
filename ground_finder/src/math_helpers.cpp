#include "math_helpers.h"

#include <algorithm>
#include <limits>

#include <geometry_msgs/Point.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <ros/ros.h>

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
    double product = 0.0;
    for (int i = 0; i < v1.size(); i++)
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
    double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
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
    if (lambda1 < 1e-9 || lambda2 < 1e-9 || lambda3 < 1e-9)
    {
        eigenvalue_ratio = 1.0;
        return false;
    }

    double sum_large = static_cast<double>(lambda1) + static_cast<double>(lambda2); // sum of the two largest eigenvalues
    eigenvalue_ratio = static_cast<double>(lambda3) / sum_large;                    // ratio of smallest eigenvalue to sum of two largest eigenvalues

    return eigenvalue_ratio <= eigenvalue_ratio_threshold; // true if distribution suggests ground plane, false if likely a wall
}

bool validateZMeanDeviation(const pcl::PointCloud<PointType>::Ptr &cloud,
                            double robot_z,
                            double max_z_deviation,
                            double &z_mean)
{
    if (!cloud || cloud->points.empty())
    {
        z_mean = robot_z;
        return false;
    }

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
        z_mean = robot_z;
        return false;
    }

    z_mean = z_sum / static_cast<double>(valid_count);
    double mean_deviation = std::abs(z_mean - robot_z);
    double z_spread = z_max - z_min;
    double max_z_spread = max_z_deviation * 2.0;

    if (mean_deviation > max_z_deviation)
    {
        return false;
    }

    if (z_spread > max_z_spread)
    {
        return false;
    }

    return true;
}

bool computeConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                             geometry_msgs::Point &hull_center)
{
    if (!cloud || cloud->points.empty())
    {
        return false;
    }

    double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
    size_t valid_count = 0;

    for (const auto &pt : cloud->points)
    {
        if (pcl::isFinite(pt))
        {
            x_sum += pt.x;
            y_sum += pt.y;
            z_sum += pt.z;
            valid_count++;
        }
    }

    if (valid_count == 0)
    {
        return false;
    }

    hull_center.x = x_sum / static_cast<double>(valid_count);
    hull_center.y = y_sum / static_cast<double>(valid_count);
    hull_center.z = z_sum / static_cast<double>(valid_count);
    return true;
}

bool validateConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                              const geometry_msgs::Point &robot_pose,
                              double max_hull_distance,
                              double &hull_distance,
                              geometry_msgs::Point &hull_center)
{
    if (!computeConvexHullCenter(cloud, hull_center))
    {
        hull_distance = std::numeric_limits<double>::max();
        return false;
    }

    double dx = hull_center.x - robot_pose.x;
    double dy = hull_center.y - robot_pose.y;
    double dz = hull_center.z - robot_pose.z;
    hull_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return hull_distance <= max_hull_distance;
}

// void calculate_mean(const std::vector<std::vector<double>> &list_vectors, std::vector<double> &mean){
//     mean = {0.0, 0.0, 0.0};
//     for(int i = 0; i < list_vectors.size(); i++){
//         mean[0] += list_vectors[i][0];
//         mean[1] += list_vectors[i][1];
//         mean[2] += list_vectors[i][2];
//     }
//     mean[0] = mean[0] / list_vectors.size();
//     mean[1] = mean[1] / list_vectors.size();
//     mean[2] = mean[2] / list_vectors.size();
// }
