#ifndef UKF_FILTER_H
#define UKF_FILTER_H

#include <vector>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include "filter.hpp"
#include "utils.hpp"

// Unscented Kalman Filter Class
class UnscentedKalmanFilter : public AttitudeEstimator {
public:
    UnscentedKalmanFilter(double alpha = 0.1, double beta = 2.0, double kappa = 0.0,
              double sigma_g = 0.01, double sigma_a = 0.1, double Pk_init = 0.1);
    
    ~UnscentedKalmanFilter();
    
    // Update filter with new IMU data
    // Returns updated quaternion
    quaternion filter(const Vec3& gyro, const Vec3& acc, float dt) override;
    
    // Reset filter state
    void reset();
    
    // Get covariance matrix (4x4)
    void getCovariance(double* P_out) const;
    
private:
    // UKF parameters
    double alpha_;       // Spread parameter
    double beta_;        // Distribution parameter
    double kappa_;       // Secondary scaling parameter
    double lambda_;      // Computed lambda parameter
    
    // Filter parameters
    double sigma_g_;     // Gyroscope noise std dev
    double sigma_a_;     // Accelerometer noise std dev
    
    // State dimension
    static const int n_ = 4;  // Quaternion has 4 elements
    static const int num_sigma_points_ = 2 * 4 + 1;  // 2n + 1
    
    // Weights for sigma points
    Eigen::VectorXd weight_mean_;
    Eigen::VectorXd weight_cov_;
    
    // State variables
    // Quaternion q0, q1, q2, q3 inherited from AttitudeEstimator
    Eigen::Matrix4d P_;             // Covariance matrix
    Vec3 g_ref_;                    // Reference gravity vector
    Eigen::Matrix4d Q_process_;     // Process noise covariance
    Eigen::Matrix3d R_measurement_; // Measurement noise covariance
    
    // Helper functions
    quaternion quaternion_from_accelerometer(const Vec3& acc) const;
    
    // UKF helper functions
    void compute_weights();
    Eigen::MatrixXd compute_sigma_points(const Eigen::Vector4d& state, const Eigen::Matrix4d& covariance);
    Eigen::Vector4d process_model(const Eigen::Vector4d& q_prev, const Vec3& omega, double dt) const;
    Eigen::Vector3d measurement_model(const Eigen::Vector4d& q, const Vec3& g_ref) const;
    
    // Matrix operations
    Eigen::Matrix4d omega_matrix(const Vec3& omega) const;
};

#endif // UKF_FILTER_H
