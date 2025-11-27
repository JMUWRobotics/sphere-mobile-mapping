#include "ekf.hpp"
#include <cstring>

// EKFFilter implementation
ExtendedKalmanFilter::ExtendedKalmanFilter(double sigma_g, double sigma_a, double Pk_init)
    : sigma_g_(sigma_g), sigma_a_(sigma_a)
{
    // Initialize quaternion state (inherited from AttitudeEstimator)
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    
    g_ref_ = Vec3(0, 0, 1);
    
    // Initialize covariance matrix P as identity * Pk_init
    std::memset(P_, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; ++i) {
        P_[i * 4 + i] = Pk_init;
    }
    
    // Initialize measurement noise covariance R
    std::memset(R_meas_, 0, 9 * sizeof(double));
    double sigma_a_sq = sigma_a * sigma_a;
    R_meas_[0] = sigma_a_sq;
    R_meas_[4] = sigma_a_sq;
    R_meas_[8] = sigma_a_sq;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {
}

void ExtendedKalmanFilter::reset() {
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    
    double Pk_init = P_[0]; // Save initial value
    std::memset(P_, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; ++i) {
        P_[i * 4 + i] = Pk_init;
    }
}

void ExtendedKalmanFilter::getCovariance(double* P_out) const {
    std::memcpy(P_out, P_, 16 * sizeof(double));
}

quaternion ExtendedKalmanFilter::quaternion_from_accelerometer(const Vec3& acc) const {
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

void ExtendedKalmanFilter::skew(const Vec3& v, double* S) const {
    // 3x3 skew-symmetric matrix
    std::memset(S, 0, 9 * sizeof(double));
    S[1] = -v.z; S[2] = v.y;
    S[3] = v.z;  S[5] = -v.x;
    S[6] = -v.y; S[7] = v.x;
}

void ExtendedKalmanFilter::omega_matrix(const Vec3& omega, double* Omega) const {
    // 4x4 matrix for quaternion propagation
    double wx = omega.x, wy = omega.y, wz = omega.z;
    std::memset(Omega, 0, 16 * sizeof(double));
    
    Omega[0] = 0.0;   Omega[1] = -wx;   Omega[2] = -wy;   Omega[3] = -wz;
    Omega[4] = wx;    Omega[5] = 0.0;   Omega[6] = wz;    Omega[7] = -wy;
    Omega[8] = wy;    Omega[9] = -wz;   Omega[10] = 0.0;  Omega[11] = wx;
    Omega[12] = wz;   Omega[13] = wy;   Omega[14] = -wx;  Omega[15] = 0.0;
}

void ExtendedKalmanFilter::f(double qw, double qx, double qy, double qz, const Vec3& omega, double dt, 
             double& q_new_w, double& q_new_x, double& q_new_y, double& q_new_z) const {
    // q_new = (I + 0.5*dt*Omega(omega)) * q_prev
    double Omega[16];
    omega_matrix(omega, Omega);
    
    double half_dt = 0.5 * dt;
    q_new_w = qw + half_dt * (Omega[0] * qw + Omega[1] * qx + Omega[2] * qy + Omega[3] * qz);
    q_new_x = qx + half_dt * (Omega[4] * qw + Omega[5] * qx + Omega[6] * qy + Omega[7] * qz);
    q_new_y = qy + half_dt * (Omega[8] * qw + Omega[9] * qx + Omega[10] * qy + Omega[11] * qz);
    q_new_z = qz + half_dt * (Omega[12] * qw + Omega[13] * qx + Omega[14] * qy + Omega[15] * qz);
}

void ExtendedKalmanFilter::dfdq(const Vec3& omega, double dt, double* F) const {
    // F = I + 0.5*dt*Omega(omega)
    double Omega[16];
    omega_matrix(omega, Omega);
    
    std::memset(F, 0, 16 * sizeof(double));
    double half_dt = 0.5 * dt;
    for (int i = 0; i < 4; ++i) {
        F[i * 4 + i] = 1.0;
        for (int j = 0; j < 4; ++j) {
            F[i * 4 + j] += half_dt * Omega[i * 4 + j];
        }
    }
}

void ExtendedKalmanFilter::dfdomega(double qw, double qx, double qy, double qz, double dt, double* W) const {
    // W = 0.5*dt * [[-q_vec^T], [q0*I + skew(q_vec)]]
    // W is 4x3 matrix
    double qv[3] = {qx, qy, qz};
    Vec3 q_vec(qx, qy, qz);
    
    double S[9];
    skew(q_vec, S);
    
    double half_dt = 0.5 * dt;
    
    // Top row: -q_vec^T
    W[0] = -half_dt * qv[0];
    W[1] = -half_dt * qv[1];
    W[2] = -half_dt * qv[2];
    
    // Bottom 3x3: q0*I + skew(q_vec)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double val = S[i * 3 + j];
            if (i == j) val += qw;
            W[(i + 1) * 3 + j] = half_dt * val;
        }
    }
}

void ExtendedKalmanFilter::h(double qw, double qx, double qy, double qz, const Vec3& g_ref, Vec3& y_pred) const {
    // Rotate global gravity to body frame using quaternion [w,x,y,z]
    y_pred.x = 2.0 * (qx * qz - qw * qy);
    y_pred.y = 2.0 * (qy * qz + qw * qx);
    y_pred.z = 1.0 - 2.0 * (qx * qx + qy * qy);
}

void ExtendedKalmanFilter::dhdq(double qw, double qx, double qy, double qz, const Vec3& g_ref, double* H) const {
    // Jacobian of h(q) w.r.t. q (3x4 matrix)
    double gx = g_ref.x, gy = g_ref.y, gz = g_ref.z;
    
    // Row 0
    H[0] = 2.0 * (gx * qw + gy * qz - gz * qy);
    H[1] = 2.0 * (gx * qx + gy * qy + gz * qz);
    H[2] = 2.0 * (-gx * qy + gy * qx - gz * qw);
    H[3] = 2.0 * (-gx * qz + gy * qw + gz * qx);
    
    // Row 1
    H[4] = 2.0 * (-gx * qz + gy * qw + gz * qx);
    H[5] = 2.0 * (gx * qy - gy * qx + gz * qw);
    H[6] = 2.0 * (gx * qx + gy * qy + gz * qz);
    H[7] = 2.0 * (-gx * qw - gy * qz + gz * qy);
    
    // Row 2
    H[8] = 2.0 * (gx * qy - gy * qx + gz * qw);
    H[9] = 2.0 * (gx * qz - gy * qw - gz * qx);
    H[10] = 2.0 * (gx * qw + gy * qz - gz * qy);
    H[11] = 2.0 * (gx * qx + gy * qy + gz * qz);
}

void ExtendedKalmanFilter::matmul_4x4(const double* A, const double* B, double* C) const {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i * 4 + j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                C[i * 4 + j] += A[i * 4 + k] * B[k * 4 + j];
            }
        }
    }
}

void ExtendedKalmanFilter::matmul_4x4_4x4T(const double* A, const double* B, double* C) const {
    // C = A * B^T
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i * 4 + j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                C[i * 4 + j] += A[i * 4 + k] * B[j * 4 + k];
            }
        }
    }
}

void ExtendedKalmanFilter::matmul_3x4_4x4(const double* A, const double* B, double* C) const {
    // C = A(3x4) * B(4x4) -> C(3x4)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i * 4 + j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                C[i * 4 + j] += A[i * 4 + k] * B[k * 4 + j];
            }
        }
    }
}

void ExtendedKalmanFilter::matmul_3x4_4x4_4x3T(const double* A, const double* B, const double* C, double* result) const {
    // result = A(3x4) * B(4x4) * C^T(3x4) -> result(3x3)
    double temp[12]; // 3x4
    matmul_3x4_4x4(A, B, temp);
    
    // Now multiply temp(3x4) * C^T(4x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i * 3 + j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                result[i * 3 + j] += temp[i * 4 + k] * C[j * 4 + k];
            }
        }
    }
}

void ExtendedKalmanFilter::matmul_4x4_4x3T(const double* A, const double* B, double* C) const {
    // C = A(4x4) * B^T(4x3) -> C(4x3)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i * 3 + j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                C[i * 3 + j] += A[i * 4 + k] * B[j * 4 + k];
            }
        }
    }
}

void ExtendedKalmanFilter::inv_3x3(const double* A, double* A_inv) const {
    // Compute inverse of 3x3 matrix using cofactor method
    double det = A[0] * (A[4] * A[8] - A[5] * A[7])
               - A[1] * (A[3] * A[8] - A[5] * A[6])
               + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    if (std::abs(det) < 1e-12) {
        // Singular matrix, return identity
        std::memset(A_inv, 0, 9 * sizeof(double));
        A_inv[0] = A_inv[4] = A_inv[8] = 1.0;
        return;
    }
    
    double inv_det = 1.0 / det;
    
    A_inv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    A_inv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    A_inv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    A_inv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    A_inv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    A_inv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    A_inv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    A_inv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    A_inv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
}

quaternion ExtendedKalmanFilter::filter(const Vec3& gyro, const Vec3& acc, float dt) {
    if (dt <= 0.0) return quaternion(q0, q1, q2, q3);
    
    Vec3 a_meas = acc;
    Vec3 omega = gyro;
    
    // Check if accelerometer measurement is valid
    double a_norm = std::sqrt(a_meas.x*a_meas.x + a_meas.y*a_meas.y + a_meas.z*a_meas.z);
    bool valid_accel = (a_norm > 0.0);
    
    Vec3 z;
    if (valid_accel) {
        z.x = a_meas.x / a_norm;
        z.y = a_meas.y / a_norm;
        z.z = a_meas.z / a_norm;
    }
    
    // ===== PREDICTION =====
    double q_pred_w, q_pred_x, q_pred_y, q_pred_z;
    f(q0, q1, q2, q3, omega, dt, q_pred_w, q_pred_x, q_pred_y, q_pred_z);
    normalizeQuat(&q_pred_w, &q_pred_x, &q_pred_y, &q_pred_z);
    
    double F[16];
    dfdq(omega, dt, F);
    
    double W[12]; // 4x3
    dfdomega(q0, q1, q2, q3, dt, W);
    
    // Q_proc = sigma_g^2 * (W * W^T)
    double WWT[16]; // 4x4
    std::memset(WWT, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) {
                WWT[i * 4 + j] += W[i * 3 + k] * W[j * 3 + k];
            }
        }
    }
    
    double sigma_g_sq = sigma_g_ * sigma_g_;
    for (int i = 0; i < 16; ++i) {
        WWT[i] *= sigma_g_sq;
    }
    
    // P_pred = F * P * F^T + Q_proc
    double FP[16];
    matmul_4x4(F, P_, FP);
    double P_pred[16];
    matmul_4x4_4x4T(FP, F, P_pred);
    
    for (int i = 0; i < 16; ++i) {
        P_pred[i] += WWT[i];
    }
    
    // ===== CORRECTION =====
    if (valid_accel) {
        Vec3 y_pred;
        h(q_pred_w, q_pred_x, q_pred_y, q_pred_z, g_ref_, y_pred);
        
        double H[12]; // 3x4
        dhdq(q_pred_w, q_pred_x, q_pred_y, q_pred_z, g_ref_, H);
        
        // Innovation: v = z - y_pred
        Vec3 v;
        v.x = z.x - y_pred.x;
        v.y = z.y - y_pred.y;
        v.z = z.z - y_pred.z;
        
        // S = H * P_pred * H^T + R_meas
        double S[9];
        matmul_3x4_4x4_4x3T(H, P_pred, H, S);
        for (int i = 0; i < 9; ++i) {
            S[i] += R_meas_[i];
        }
        
        // K = P_pred * H^T * inv(S)
        double S_inv[9];
        inv_3x3(S, S_inv);
        
        double PHT[12]; // 4x3
        matmul_4x4_4x3T(P_pred, H, PHT);
        
        double K[12]; // 4x3
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                K[i * 3 + j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    K[i * 3 + j] += PHT[i * 3 + k] * S_inv[k * 3 + j];
                }
            }
        }
        
        // Update quaternion: q = q_pred + K * v
        q0 = q_pred_w + K[0] * v.x + K[1] * v.y + K[2] * v.z;
        q1 = q_pred_x + K[3] * v.x + K[4] * v.y + K[5] * v.z;
        q2 = q_pred_y + K[6] * v.x + K[7] * v.y + K[8] * v.z;
        q3 = q_pred_z + K[9] * v.x + K[10] * v.y + K[11] * v.z;
        normalizeQuat(&q0, &q1, &q2, &q3);
        
        // Update covariance: P = (I - K*H) * P_pred
        double KH[16]; // 4x4
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                KH[i * 4 + j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    KH[i * 4 + j] += K[i * 3 + k] * H[k * 4 + j];
                }
            }
        }
        
        double I_KH[16];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                I_KH[i * 4 + j] = (i == j ? 1.0 : 0.0) - KH[i * 4 + j];
            }
        }
        
        matmul_4x4(I_KH, P_pred, P_);
    } else {
        // No correction, just use prediction
        q0 = q_pred_w;
        q1 = q_pred_x;
        q2 = q_pred_y;
        q3 = q_pred_z;
        std::memcpy(P_, P_pred, 16 * sizeof(double));
    }
    
    return quaternion(q0, q1, q2, q3);
}
