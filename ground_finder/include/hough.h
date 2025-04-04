/*
 * hough.h
 * Implementation of the Randomized Hough Transform incl. ball accumulator based on slam6d
 *
 * Author: Carolin Bösch
 */

#ifndef HOUGH_H
#define HOUGH_H

#include "math_helpers.h"
#include <pcl/common/projection_matrix.h>

/* Ball accumulator class */
class Accumulator {
private:
    std::vector<std::vector<std::vector<int>>> accumulator;
    std::vector<int> ballNr;
    int rhoNum, phiNum;
    int rhoMax, accumulatorMax;

public:
    /** \brief Constructor of ball accumulator object
     * \param[in] rhoNum   Number of cells in direction of ρ
     * \param[in] phiNum   Number of cells in direction of φ
     * \param[in] thetaNum Number of cells in direction of θ
     * \param[in] rhoMax   Limitation of range of ρ values in the Hough space
     * \param[in] accumulatorMax Maximal accumulator count; Any cell in accumulator > accumulatorMax = potential plane candidate.
     */
    Accumulator(int rhoNum, int phiNum, int thetaNum, int rhoMax, int accumulatorMax);

    /** \brief Accumulates votes in the accumulator array for a given plane defined by theta, phi, and rho.
     * \param[in] theta Angle theta of the plane in radians
     * \param[in] phi   Angle phi of the plane in radians
     * \param[in] rho   Distance rho of the plane from the origin
     * \return True if the accumuated cell > accumulatorMax, false otherwise.
     */
    bool accumulate(double theta, double phi, double rho);

    /** \brief Resets the accumulator array to zero.
     */
    void resetAccumulator();
};


/* Hough class */
class Hough {
private:
    pcl::PointCloud<PointType>::Ptr scan;
    Accumulator* acc;
    double minDist, maxDist;

public:
    /** \brief Constructor of Hough object
     * \param[in] scan Pointer to the input point cloud
     * \param[in] acc Pointer to the accumulator object
     * \param[in] minDist Minimum distance for plane fitting
     * \param[in] maxDist Maximum distance for plane fitting
     */
    Hough(pcl::PointCloud<PointType>::Ptr &scan, Accumulator* acc, int minDist, int maxDist);

    /** \brief Checks if the distance between three points is within a valid range (set by minDst and maxDist) for plane fitting.
     * \param[in] p1 First point
     * \param[in] p2 Second point
     * \param[in] p3 Third point
     * \return True if the distance is within the valid range, false otherwise.
     */
    bool distanceOK(PointType p1, PointType p2, PointType p3);

    /** \brief Calculates the parameters (theta, phi, and rho) of a plane passing through three points.
     * \param[in] p1 First point
     * \param[in] p2 Second point
     * \param[in] p3 Third point
     * \param[out] theta Angle theta of the plane in radians
     * \param[out] phi Angle phi of the plane in radians
     * \param[out] rho Distance rho of the plane from the origin
     * \return True if the plane parameters are successfully calculated, false otherwise.
     */
    bool calculatePlane(PointType p1, PointType p2, PointType p3, double &theta, double &phi, double &rho);

    /** \brief Computes the normal vector of a plane given its parameters (rho, theta, phi).
     * \param[out] n Normal vector of the plane
     * \param[in] rho Distance rho of the plane from the origin
     * \param[in] theta Angle theta of the plane in radians
     * \param[in] phi Angle phi of the plane in radians
     */
    void computeNormalVector(std::vector<double> &n, double rho, double theta, double phi);

    /** \brief Runs the Randomized Hough Transform (RHT) algorithm to detect the dominant plane in \a scan.
     * \param[out] n Normal vector of the detected plane.
     * \return Plane Parameter rho
     */
    double RHT(std::vector<double> &n);
};

#endif
