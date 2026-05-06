#include "mahony.hpp"
#include <cstring>


// MahonyFilter implementation
MahonyFilter::MahonyFilter(double k_P, double k_I)
    : k_P_(k_P), k_I_(k_I)
{
    // Initialize quaternion state (inherited from AttitudeEstimator)
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    bias_ = Vec3(0, 0, 0);
}


Vec3 MahonyFilter::measurement_gravity(const quaternion& q) const {
    // Expected gravity in body frame for q=[w,x,y,z]
    // Must match Python implementation exactly
    Vec3 g;
    g.x = 2.0 * (q.x * q.z - q.w * q.y);
    g.y = 2.0 * (q.y * q.z + q.w * q.x);
    g.z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return g;
}

quaternion MahonyFilter::quaternion_from_accelerometer(const Vec3& acc) const {
    double norm = std::sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    if (norm == 0.0) return quaternion(1, 0, 0, 0);
    
    double ax = acc.x / norm;
    double ay = acc.y / norm;
    double az = acc.z / norm;
    
    double roll = std::atan2(ay, az);
    double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    
    // Convert to quaternion (ZYX convention)
    double cy = std::cos(0.0 * 0.5);
    double sy = std::sin(0.0 * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    
    quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

quaternion MahonyFilter::filter(const Vec3& gyro, const Vec3& acc, float dt) {
    //if (dt <= 0.0) return q_;
    
    Vec3 a_meas(acc.x, acc.y, acc.z);
        
    // Normalize acceleration measurement
    double a_norm = std::sqrt(a_meas.x*a_meas.x + a_meas.y*a_meas.y + a_meas.z*a_meas.z);
    
    Vec3 omega_corr = gyro;
    
    if (a_norm > 0.0) {
        // Unit acceleration vector
        Vec3 a_unit(a_meas.x / a_norm, a_meas.y / a_norm, a_meas.z / a_norm);
        
        // Expected gravity direction from current quaternion estimate
        // Must match Python implementation exactly
        Vec3 a_exp;
        a_exp.x = 2.0 * (q1 * q3 - q0 * q2);
        a_exp.y = 2.0 * (q2 * q3 + q0 * q1);
        a_exp.z = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        
        // Error: cross product (measured x expected) - consistent with Python
        Vec3 e;
        e.x = a_unit.y * a_exp.z - a_unit.z * a_exp.y;
        e.y = a_unit.z * a_exp.x - a_unit.x * a_exp.z;
        e.z = a_unit.x * a_exp.y - a_unit.y * a_exp.x;
        
        // Integrate bias
        bias_.x += k_I_ * e.x * dt;
        bias_.y += k_I_ * e.y * dt;
        bias_.z += k_I_ * e.z * dt;
        
        // Corrected angular rate: add proportional error correction, subtract bias
        omega_corr.x = gyro.x + k_P_ * e.x - bias_.x;
        omega_corr.y = gyro.y + k_P_ * e.y - bias_.y;
        omega_corr.z = gyro.z + k_P_ * e.z - bias_.z;
    } else {
        // No correction, just remove bias
        omega_corr.x = gyro.x - bias_.x;
        omega_corr.y = gyro.y - bias_.y;
        omega_corr.z = gyro.z - bias_.z;
    }
    
    // Quaternion propagation using corrected angular rate
    double wx = omega_corr.x;
    double wy = omega_corr.y;
    double wz = omega_corr.z;
    double half_dt = 0.5 * dt;
    
    // Store current quaternion
    double q0_prev = q0;
    double q1_prev = q1;
    double q2_prev = q2;
    double q3_prev = q3;
    
    // Compute new quaternion using previous values (not accumulating)
    q0 = q0_prev + half_dt * (-wx * q1_prev - wy * q2_prev - wz * q3_prev);
    q1 = q1_prev + half_dt * ( wx * q0_prev + wz * q2_prev - wy * q3_prev);
    q2 = q2_prev + half_dt * ( wy * q0_prev - wz * q1_prev + wx * q3_prev);
    q3 = q3_prev + half_dt * ( wz * q0_prev + wy * q1_prev - wx * q2_prev);
    
    // Normalize
    normalizeQuat(&q0, &q1, &q2, &q3);
    
    return quaternion(q0, q1, q2, q3);
}
