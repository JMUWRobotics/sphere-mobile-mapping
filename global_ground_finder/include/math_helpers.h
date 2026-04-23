/*
 * Author: Carolin Bösch
 */

#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <cmath>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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

/** \brief Validates point distribution from PCA eigenvalues (ground plane check)
 * Ground planes should have 2 large eigenvalues (XY spread) and 1 small (Z thickness).
 * Walls have more uniform eigenvalue distribution.
 * \param[in] lambda1 Largest eigenvalue
 * \param[in] lambda2 Middle eigenvalue
 * \param[in] lambda3 Smallest eigenvalue
 * \param[in] eigenvalue_ratio_threshold Threshold for (lambda3 / (lambda1 + lambda2))
 * \param[out] eigenvalue_ratio The computed ratio for logging/debugging
 * \return true if distribution suggests ground plane, false if likely a wall
 */
bool validatePointDistributionFromEigenvalues(float lambda1, float lambda2, float lambda3,
                                              double eigenvalue_ratio_threshold,
                                              double &eigenvalue_ratio);

/** \brief Validates Z-coordinate mean relative to robot pose (ground plane check)
 * Ground planes should have points distributed around the robot's Z coordinate.
 * Large deviations suggest ceiling or other non-ground planes.
 * \param[in] cloud Point cloud of inliers
 * \param[in] robot_z Z coordinate of robot pose
 * \param[in] max_z_deviation Maximum allowed deviation from robot Z
 * \param[out] z_mean The computed mean Z coordinate
 * \return true if Z distribution is reasonable for ground, false otherwise
 */
bool validateZMeanDeviation(const pcl::PointCloud<PointType>::Ptr &cloud,
                            double robot_z,
                            double max_z_deviation,
                            double &z_mean);

/** \brief Calculates the convex hull of a point cloud and returns its center point
 * \param[in] cloud Point cloud
 * \param[out] hull_center Center point of the convex hull (mean of hull vertices)
 * \return true if hull calculated successfully, false otherwise
 */
bool computeConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                             geometry_msgs::Point &hull_center);

/** \brief Validates convex hull center relative to robot pose
 * Ground plane should have its center spatially close to cur pose
 * Ceiling/walls have center far from robot
 * \param[in] cloud Point cloud of inliers
 * \param[in] robot_pose Robot position (x, y, z)
 * \param[in] max_hull_distance Maximum allowed 3D distance from robot to hull center
 * \param[out] hull_distance 3D Euclidean distance from robot to hull center
 * \return true if hull center is within max_hull_distance, false otherwise
 */
bool validateConvexHullCenter(const pcl::PointCloud<PointType>::Ptr &cloud,
                              const geometry_msgs::Point &robot_pose,
                              double max_hull_distance,
                              double &hull_distance,
                              geometry_msgs::Point &hull_center);

#endif
