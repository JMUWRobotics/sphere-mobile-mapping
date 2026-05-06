#include "ukf.hpp"
#include <iostream>

// Define static members
const int UnscentedKalmanFilter::n_;
const int UnscentedKalmanFilter::num_sigma_points_;

// UKFFilter implementation
UnscentedKalmanFilter::UnscentedKalmanFilter(double alpha, double beta, double kappa,
                     double sigma_g, double sigma_a, double Pk_init)
    : alpha_(alpha), beta_(beta), kappa_(kappa),
      sigma_g_(sigma_g), sigma_a_(sigma_a)
{
    // Initialize quaternion state (inherited from AttitudeEstimator)
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    
    g_ref_ = Vec3(0, 0, 1);
    
    // Compute lambda parameter: λ = alpha^2(n + kappa) - n
    lambda_ = alpha * alpha * (n_ + kappa) - n_;
    
    // Initialize covariance matrix P with larger initial uncertainty
    P_ = Eigen::Matrix4d::Identity() * std::max(Pk_init, 0.1);
    
    // Initialize process noise covariance Q - should reflect actual gyro drift
    // Using dt=0.01 as nominal: Q = sigma_g^2 * dt^2 * I
    Q_process_ = Eigen::Matrix4d::Identity() * sigma_g * sigma_g * 0.01;
    
    // Initialize measurement noise covariance R
    R_measurement_ = Eigen::Matrix3d::Identity() * sigma_a * sigma_a;
    
    // Compute weights
    compute_weights();
}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {
}

void UnscentedKalmanFilter::reset() {
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    
    double Pk_init = P_(0, 0); // Save initial value
    P_ = Eigen::Matrix4d::Identity() * Pk_init;
}

void UnscentedKalmanFilter::getCovariance(double* P_out) const {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P_out[i * 4 + j] = P_(i, j);
        }
    }
}

void UnscentedKalmanFilter::compute_weights() {
    weight_mean_.resize(num_sigma_points_);
    weight_cov_.resize(num_sigma_points_);
    
    // Weight for the mean sigma point
    weight_mean_(0) = lambda_ / (n_ + lambda_);
    weight_cov_(0) = weight_mean_(0) + (1.0 - alpha_ * alpha_ + beta_);
    
    // Weights for remaining sigma points
    for (int i = 1; i < num_sigma_points_; ++i) {
        weight_mean_(i) = 1.0 / (2.0 * (n_ + lambda_));
        weight_cov_(i) = weight_mean_(i);
    }
}

Eigen::MatrixXd UnscentedKalmanFilter::compute_sigma_points(const Eigen::Vector4d& state, const Eigen::Matrix4d& covariance) {
    Eigen::MatrixXd sigma_points(n_, num_sigma_points_);
    
    // Ensure covariance is positive definite by adding small regularization
    Eigen::Matrix4d P_reg = covariance + Eigen::Matrix4d::Identity() * 1e-9;
    
    // Try Cholesky decomposition
    Eigen::Matrix4d scaled_cov = (n_ + lambda_) * P_reg;
    
    // Ensure scaled covariance is positive by checking lambda
    if (n_ + lambda_ <= 0) {
        std::cerr << "Warning: n + lambda = " << (n_ + lambda_) << " is non-positive!" << std::endl;
        // Use absolute value or default scaling
        scaled_cov = std::abs(n_ + lambda_) * P_reg;
        if (scaled_cov.trace() < 1e-6) {
            scaled_cov += Eigen::Matrix4d::Identity() * 1e-6;
        }
    }
    
    Eigen::LLT<Eigen::Matrix4d> llt(scaled_cov);
    
    Eigen::Matrix4d L;
    if (llt.info() == Eigen::Success) {
        L = llt.matrixL();
    } else {
        // Cholesky failed, use eigenvalue decomposition for robustness
        std::cerr << "Cholesky decomposition failed, using eigenvalue decomposition" << std::endl;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(scaled_cov);
        
        // Make sure all eigenvalues are positive
        Eigen::Vector4d eigenvalues = es.eigenvalues();
        for (int i = 0; i < 4; ++i) {
            if (eigenvalues(i) < 1e-9) {
                eigenvalues(i) = 1e-9;
            }
        }
        
        // Reconstruct matrix square root: L = V * sqrt(D)
        L = es.eigenvectors() * eigenvalues.cwiseSqrt().asDiagonal();
    }
    
    // First sigma point is the mean
    sigma_points.col(0) = state;
    
    // Generate remaining sigma points
    for (int i = 0; i < n_; ++i) {
        sigma_points.col(i + 1) = state + L.col(i);
        sigma_points.col(n_ + i + 1) = state - L.col(i);
    }
    
    return sigma_points;
}

Eigen::Matrix4d UnscentedKalmanFilter::omega_matrix(const Vec3& omega) const {
    double wx = omega.x, wy = omega.y, wz = omega.z;
    Eigen::Matrix4d Omega;
    
    Omega << 0.0,  -wx,  -wy,  -wz,
             wx,   0.0,   wz,  -wy,
             wy,  -wz,   0.0,   wx,
             wz,   wy,  -wx,   0.0;
    
    return Omega;
}

Eigen::Vector4d UnscentedKalmanFilter::process_model(const Eigen::Vector4d& q_prev, const Vec3& omega, double dt) const {
    // State transition using omega operator
    // Phi = I + 0.5*dt*Omega(omega)
    Eigen::Matrix4d Phi = Eigen::Matrix4d::Identity() + 0.5 * dt * omega_matrix(omega);
    
    Eigen::Vector4d q_new = Phi * q_prev;
    q_new.normalize();  // Normalize quaternion
    
    return q_new;
}

Eigen::Vector3d UnscentedKalmanFilter::measurement_model(const Eigen::Vector4d& q, const Vec3& g_ref) const {
    // Rotate reference gravity to body frame
    double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    
    Eigen::Vector3d a_expected;
    a_expected(0) = 2.0 * (qx * qz - qw * qy);
    a_expected(1) = 2.0 * (qy * qz + qw * qx);
    a_expected(2) = 1.0 - 2.0 * (qx * qx + qy * qy);
    
    return a_expected;
}

quaternion UnscentedKalmanFilter::quaternion_from_accelerometer(const Vec3& acc) const {
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

quaternion UnscentedKalmanFilter::filter(const Vec3& gyro, const Vec3& acc, float dt) {
    if (dt <= 0.0) return quaternion(q0, q1, q2, q3);
    
    Vec3 a_meas = acc;
    Vec3 omega = gyro;
    
    // Check if accelerometer measurement is valid
    double a_norm = std::sqrt(a_meas.x*a_meas.x + a_meas.y*a_meas.y + a_meas.z*a_meas.z);
    if (a_norm == 0.0) return quaternion(q0, q1, q2, q3);
    
    // Normalized measurement
    Eigen::Vector3d z;
    z(0) = a_meas.x / a_norm;
    z(1) = a_meas.y / a_norm;
    z(2) = a_meas.z / a_norm;
    
    // ===== PREDICTION STEP =====
    // 1. Generate Sigma Points
    Eigen::Vector4d q_vec(q0, q1, q2, q3);
    Eigen::MatrixXd sigma_points = compute_sigma_points(q_vec, P_);
    
    // 2. Propagate sigma points through process model
    Eigen::MatrixXd sigma_points_prop(n_, num_sigma_points_);
    for (int j = 0; j < num_sigma_points_; ++j) {
        sigma_points_prop.col(j) = process_model(sigma_points.col(j), omega, dt);
    }
    
    // 3. Predicted state mean
    Eigen::Vector4d q_pred = Eigen::Vector4d::Zero();
    for (int j = 0; j < num_sigma_points_; ++j) {
        q_pred += weight_mean_(j) * sigma_points_prop.col(j);
    }
    q_pred.normalize();  // Normalize quaternion
    
    // 4. Predicted state covariance
    Eigen::Matrix4d P_pred = Eigen::Matrix4d::Zero();
    for (int j = 0; j < num_sigma_points_; ++j) {
        Eigen::Vector4d diff = sigma_points_prop.col(j) - q_pred;
        P_pred += weight_cov_(j) * diff * diff.transpose();
    }
    P_pred += Q_process_;
    
    // ===== CORRECTION STEP =====
    // 5. Transform sigma points into measurement space
    Eigen::MatrixXd sigma_points_meas(3, num_sigma_points_);
    for (int j = 0; j < num_sigma_points_; ++j) {
        sigma_points_meas.col(j) = measurement_model(sigma_points_prop.col(j), g_ref_);
    }
    
    // 6. Predicted measurement mean
    Eigen::Vector3d z_pred = Eigen::Vector3d::Zero();
    for (int j = 0; j < num_sigma_points_; ++j) {
        z_pred += weight_mean_(j) * sigma_points_meas.col(j);
    }
    
    // 7. Measurement covariance
    Eigen::Matrix3d P_zz = Eigen::Matrix3d::Zero();
    for (int j = 0; j < num_sigma_points_; ++j) {
        Eigen::Vector3d diff_meas = sigma_points_meas.col(j) - z_pred;
        P_zz += weight_cov_(j) * diff_meas * diff_meas.transpose();
    }
    P_zz += R_measurement_;
    
    // 8. Cross-covariance
    Eigen::Matrix<double, 4, 3> P_xz = Eigen::Matrix<double, 4, 3>::Zero();
    for (int j = 0; j < num_sigma_points_; ++j) {
        Eigen::Vector4d diff_state = sigma_points_prop.col(j) - q_pred;
        Eigen::Vector3d diff_meas = sigma_points_meas.col(j) - z_pred;
        P_xz += weight_cov_(j) * diff_state * diff_meas.transpose();
    }
    
    // 9. Kalman Gain
    Eigen::Matrix<double, 4, 3> K = P_xz * P_zz.inverse();
    
    // 10. Innovation (residual)
    Eigen::Vector3d innovation = z - z_pred;
    
    // 11. Update state
    Eigen::Vector4d q_updated = q_pred + K * innovation;
    q_updated.normalize();  // Normalize quaternion
    
    // Store back to parent class state
    q0 = q_updated(0);
    q1 = q_updated(1);
    q2 = q_updated(2);
    q3 = q_updated(3);
    
    // 12. Update covariance - ensure it stays positive definite
    P_ = P_pred - K * P_zz * K.transpose();
    
    // Force symmetry
    P_ = 0.5 * (P_ + P_.transpose());
    
    // Ensure positive definiteness by checking eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(P_);
    if (es.eigenvalues().minCoeff() < 1e-9) {
        // Add small regularization if needed
        P_ += Eigen::Matrix4d::Identity() * 1e-9;
    }
    
    return quaternion(q0, q1, q2, q3);
}
