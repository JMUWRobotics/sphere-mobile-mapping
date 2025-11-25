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

// void calculate_mean(const std::vector<std::vector<double>> &list_vectors, std::vector<double> &mean);

#endif
