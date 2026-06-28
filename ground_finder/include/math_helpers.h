/*
 * math_helpers.h
 * Useful mathematical functions regarding 3D space calculations
 *
 * Author: Carolin Bösch
 */

#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <cmath>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <geometry_msgs/Point.h>

using PointType = pcl::PointXYZ;

/** \brief Converts degree to rad
 */
double deg2rad(const double deg);

/** \brief Subtract two points (p1 - p2)
 * \param[in] p1 Point1 = minuend
 * \param[in] p2 Point2 = subtrahend
 * \param[out] d The difference of the two points as a vector.
 */
void subtract_points(const PointType &p1, const PointType &p2, std::vector<double> &d);

/** \brief Calculates the dot product of two vectors (v1°v2)
 * \param[in] v1 First vector
 * \param[in] v2 Second vector
 */
double dot_product(const std::vector<double> &v1, const std::vector<double> &v2);

/** \brief Calculates the cross product of two vectors (v1 x v2)
 * \param[in] v1 First vector
 * \param[in] v2 Second vector
 * \param[out] res Resulting cross product vector of v1 and v2
 */
void cross_product(const std::vector<double> &v1, const std::vector<double> &v2, std::vector<double> &res);

/** \brief Normalizes the vector v
 */
void normalize_vector(std::vector<double> &v);

/** \brief Converts a cartesian vector to polar cooridnates
 * \param[in] cartesian_vector vector to be converted
 * \return The polar coordinates as a vector (phi, theta, rho)
 */
std::vector<double> convert2polar(std::vector<double> cartesian_vector);

/** rief Validates point distribution from PCA eigenvalues (ground plane check)
 * Ground planes should have 2 large eigenvalues (XY spread) and 1 small (Z thickness).
 * Walls have more uniform eigenvalue distribution.
 *
eturn true if distribution suggests ground plane, false if likely a wall
 */
bool validatePointDistributionFromEigenvalues(float lambda1, float lambda2, float lambda3,
                                              double eigenvalue_ratio_threshold,
                                              double &eigenvalue_ratio);

/** rief Validates Z-coordinate mean relative to robot pose (ground plane check)
 */
bool validateZMeanDeviation(const pcl::PointCloud<PointType>::Ptr &cloud,
                            double robot_z,
                            double max_z_deviation,
                            double &z_mean);

/** rief Calculates the convex hull of a point cloud and returns its center point
 */
bool computeConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                             geometry_msgs::Point &hull_center);

/** rief Validates convex hull center relative to robot pose
 */
bool validateConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                              const geometry_msgs::Point &robot_pose,
                              double max_hull_distance,
                              double &hull_distance,
                              geometry_msgs::Point &hull_center);

// void calculate_mean(const std::vector<std::vector<double>> &list_vectors, std::vector<double> &mean);

#endif
