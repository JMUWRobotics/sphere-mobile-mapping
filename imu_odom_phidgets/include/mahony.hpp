#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#include "filter.hpp"
#include "utils.hpp"

// Mahony Filter Class
class MahonyFilter : public AttitudeEstimator {
public:
    MahonyFilter(double k_P = 1.0, double k_I = 0.001);
    
    // Update filter with new IMU data
    // Returns updated quaternion
    quaternion filter(const Vec3& gyro, const Vec3& acc, float dt) override;
    
private:
    // Filter gains
    double k_P_;  // Proportional gain
    double k_I_;  // Integral gain (gyro bias correction)
    
    // State variables
    // quaternion q_;           // Current orientation estimate
    Vec3 bias_;        // Gyro bias estimate
    
    // Helper functions
    Vec3 measurement_gravity(const quaternion& q) const;
    quaternion quaternion_from_accelerometer(const Vec3& acc) const;
};

#endif // MAHONY_FILTER_H
