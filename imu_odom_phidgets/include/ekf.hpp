#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <vector>
#include <cmath>
#include <cstring>

#include "filter.hpp"
#include "utils.hpp"

// Extended Kalman Filter Class
class ExtendedKalmanFilter : public AttitudeEstimator {
public:
    ExtendedKalmanFilter(double sigma_g = 0.001, double sigma_a = 0.001, double Pk_init = 0.01);
    
    ~ExtendedKalmanFilter();
    
    // Update filter with new IMU data
    // Returns updated quaternion
    quaternion filter(const Vec3& gyro, const Vec3& acc, float dt) override;
    
    // Reset filter state
    void reset();
    
    // Get covariance matrix (4x4)
    void getCovariance(double* P_out) const;
    
private:
    // Filter parameters
    double sigma_g_;     // Gyroscope noise std dev
    double sigma_a_;     // Accelerometer noise std dev
    
    // State variables
    // Quaternion q0, q1, q2, q3 inherited from AttitudeEstimator
    double P_[16];       // Covariance matrix (4x4, row-major)
    Vec3 g_ref_;         // Reference gravity vector
    double R_meas_[9];   // Measurement noise covariance (3x3)
    
    // Helper functions
    quaternion quaternion_from_accelerometer(const Vec3& acc) const;
    
    // EKF helper functions
    void f(double qw, double qx, double qy, double qz, const Vec3& omega, double dt, 
           double& q_new_w, double& q_new_x, double& q_new_y, double& q_new_z) const;
    void dfdq(const Vec3& omega, double dt, double* F) const;
    void dfdomega(double qw, double qx, double qy, double qz, double dt, double* W) const;
    void h(double qw, double qx, double qy, double qz, const Vec3& g_ref, Vec3& y_pred) const;
    void dhdq(double qw, double qx, double qy, double qz, const Vec3& g_ref, double* H) const;
    
    // Matrix operations
    void skew(const Vec3& v, double* S) const;
    void omega_matrix(const Vec3& omega, double* Omega) const;
    void matmul_4x4(const double* A, const double* B, double* C) const;
    void matmul_4x4_4x4T(const double* A, const double* B, double* C) const;
    void matmul_3x4_4x4(const double* A, const double* B, double* C) const;
    void matmul_3x4_4x4_4x3T(const double* A, const double* B, const double* C, double* result) const;
    void matmul_4x4_4x3T(const double* A, const double* B, double* C) const;
    void inv_3x3(const double* A, double* A_inv) const;
};

#endif // EKF_FILTER_H
