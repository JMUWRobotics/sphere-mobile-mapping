#ifndef IMU_JASPER_UTILS_H
#define IMU_JASPER_UTILS_H

#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <numeric>
#include <eigen3/Eigen/Dense>

// Simple 3D vector structure
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// Define quaternion in order w x y z (scalar w)
struct quaternion
{
    double  w; 
    double  x;
    double  y;
    double  z;
    quaternion() {
        // Creates identity
        w = 1;
        x = 0;
        y = 0;
        z = 0;
    };
    quaternion(double q0, double q1, double q2, double q3) : w(q0), x(q1), y(q2), z(q3) {};
    
    // Convert to/from Eigen vector
    Eigen::Vector4d toVector() const { return Eigen::Vector4d(w, x, y, z); }
    void fromVector(const Eigen::Vector4d& v) { w = v(0); x = v(1); y = v(2); z = v(3); }
};

// Quaternion operations, defined in utils.cpp

inline double quaternion_dot_product(quaternion *l, quaternion *r)
{
    return l->w * r->w + l->x * r->x + l->y * r->y + l->z * r->z;
}

inline void quaternion_normalize(quaternion *q)
{
    double norm = 1.0 / sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0)
        norm = 0.0000001;
    q->w *= norm;
    q->x *= norm;
    q->y *= norm;
    q->z *= norm;
}

template <typename T>
inline void normalizeQuat(T *q0, T *q1, T *q2, T *q3)
{
    T recipNorm = 1.0 / sqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;
}

inline void quaternion_copy(quaternion *original_q, quaternion *copy_q)
{
    copy_q->w = original_q->w;
    copy_q->x = original_q->x;
    copy_q->y = original_q->y;
    copy_q->z = original_q->z;
}

inline void quaternion_inverse(const quaternion* q, quaternion* res)
{
    res->w =  q->w;
    res->x = -q->x;
    res->y = -q->y;
    res->z = -q->z;
    quaternion_normalize(res);
}

inline void quaternion_multiply(const quaternion* q1, const quaternion* q2, quaternion* res)
{
    res->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    res->x = q1->w * q2->x + q1->x * q2->w - q1->y * q2->z + q1->z * q2->y;
    res->y = q1->w * q2->y + q1->x * q2->z + q1->y * q2->w - q1->z * q2->x;
    res->z = q1->w * q2->z - q1->x * q2->y + q1->y * q2->x + q1->z * q2->w;
    quaternion_normalize(res);
}

inline void quaternion_to_axis_angle(const quaternion* in, double* axis, double* angle) 
{
    double norm = sqrt(in->x * in->x + in->y * in->y + in->z * in->z);
    double recipNorm = 1.0 / norm;
    if (norm < 0.001) {
        *angle = 0; 
        axis[0] = 1;
        axis[1] = 0;
        axis[2] = 0;
    } else {
        *angle = 2*acos(std::min( std::max(in->w, -1.0), 1.0) );
        axis[0] = in->x * recipNorm;
        axis[1] = in->y * recipNorm;
        axis[2] = in->z * recipNorm;
    }
}

inline void vectorCross(const float* v1, const float* v2, float* res) {
    res[0] = v1[1]*v2[2] - v1[2]*v2[1];
    res[1] = v1[2]*v2[0] - v1[0]*v2[2];
    res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

inline void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw)
{
    yaw *= 0.5;
    pitch *= 0.5;
    roll *= 0.5;

    float cpsi = cos(yaw);
    float spsi = sin(yaw);
    float cth = cos(pitch);
    float sth = sin(pitch);
    float cphi = cos(roll);
    float sphi = sin(roll);

    *qW = cpsi * cth * cphi + spsi * sth * sphi;
    *qX = cpsi * cth * sphi - spsi * sth * cphi;
    *qY = cpsi * sth * cphi + spsi * cth * sphi;
    *qZ = spsi * cth * cphi - cpsi * sth * sphi;

    normalizeQuat(qW, qX, qY, qZ);
}

inline void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ)
{
    *roll = atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY);
    *pitch = asinf(-2.0f * (qX * qZ - qW * qY));
    *yaw = atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);
}

inline void vectorAsNormalized(const float *v, float* res)
{
    float v2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    float recipNorm = v2 < 0.0001 ? 0 : 1.0 / sqrt(v2);
    res[0] = v[0] * recipNorm;
    res[1] = v[1] * recipNorm;
    res[2] = v[2] * recipNorm;
}

inline void quaternion_slerp(quaternion *l, quaternion *r, quaternion *o, double weight)
{
    double dot = quaternion_dot_product(l, r);
    if (dot > 0.9995)
    {
        o->w = l->w + (r->w - l->w) * weight;
        o->x = l->x + (r->x - l->x) * weight;
        o->y = l->y + (r->y - l->y) * weight;
        o->z = l->z + (r->z - l->z) * weight;
        quaternion_normalize(o);
        return;
    }

    if (dot > 1)
        dot = 1;
    else if (dot < -1)
        dot = -1;

    double theta_0 = acos(dot);
    double theta = (0. < theta_0 && theta_0 < M_PI_2) ? theta_0 * weight : (theta_0 - M_PI) * weight;

    o->w = r->w - l->w * dot;
    o->x = r->x - l->x * dot;
    o->y = r->y - l->y * dot;
    o->z = r->z - l->z * dot;

    quaternion_normalize(o);

    o->w = l->w * cos(theta) + o->w * sin(theta);
    o->x = l->x * cos(theta) + o->x * sin(theta);
    o->y = l->y * cos(theta) + o->y * sin(theta);
    o->z = l->z * cos(theta) + o->z * sin(theta);
}

// Implements a passive time synchronization scheme
class passive_time_sync
{
    public:
    // Default constructor initializes all time syncronization style
    passive_time_sync() {
        // Default case assumes that the sensor clock could be up to 5 percent slower or 
        // faster than the master clock.
        alpha_1 = 0.05;
        alpha_2 = 0.05;
        
        // Set bound of clock drift when time difference between measurements 
        // is zero, to zero.
        f0 = 0.0;
        
        // Start Online Syncing
        start_online = true;
    };
    
    // Parameterized constructor to create time syncronization object
    // Inputs:   alpha_1_input = The extent to which the sensor's clock may be slow
    //           alpha_2_input = The extent to which the sensor's clock may be fast
    //           f0_input = Set bound of clock drift when time difference between 
    //                      measurements is zero.
    // Outputs: Assigns Class's private parameters
    passive_time_sync(double alpha_1_input, double alpha_2_input, double f0_input) {
        // Set assumptions on how fast or slow the sensor clock may be relative to 
        // the master clock.
        alpha_1 = alpha_1_input;
        alpha_2 = alpha_2_input;
        
        // Set bound of clock drift when time difference between measurements is zero.
        f0 = f0_input;
        
        // Start Online Syncing
        start_online = true;
    };
    
    // Default destructor eliminates all object parameter
    ~passive_time_sync(){};
    
    // Get Online Sync is called every time the master computer receives a sensor 
    // measurement. Sensor measurement time stamps are synced with the master clock.
    // Inputs: sens_time_stamps = a sensor time stamp
    //         master_time_stamps = a master clock time stamp
    // Outputs: corrected sensor time stamp
    double online_sync(double sens_time_stamps, double master_time_stamps) {
        // Initialize the relevant variables 
        double synced_time;
        double A_offset;
        
        // On the first call of the online sync, assign two variables some initial values
        if (start_online == true)
        {
            p_online = sens_time_stamps;
            q_online = master_time_stamps;
            start_online = false;
        }
        
        // Check for the shortest latency time (accounting for clock drift) 
        if (sens_time_stamps - master_time_stamps - f0 >= p_online - q_online - clock_func(abs(sens_time_stamps - p_online)))
        {
            p_online = sens_time_stamps;
            q_online = master_time_stamps;
            A_offset = p_online - q_online - f0;
        }
        else
        {
            A_offset = p_online - q_online - clock_func(abs(sens_time_stamps - p_online));
        }
        
        // Fix the time offset
        synced_time = sens_time_stamps - A_offset;
        
        // Return Result
        return synced_time;
    };
    
    private:
    // Set of sensor measurement times
    double p_online; 
    
    // Set of master clock times at which 
    // sensor measurement times are received
    double q_online;
    
    // The extent to which the sensor's clock may be slow
    double alpha_1;
    
    // The extent to which the sensor's clock may be fast
    double alpha_2;
    
    // The initial clock drift bound when the time difference 
    // between measurements is zero.
    double f0;
    
    // A flag to identify if the online synchronization has begun
    bool start_online;
    
    // This is the drift model for the clock
    // Input:  dp = change in time between sensor time stamps
    // Output: Upper bound of the change in offset 
    double clock_func(double dp) {
        // Initialize output
        double output;
        // Choose the drift model for the clock
        if (alpha_1 >= alpha_2) {
            output = dp*alpha_1 + f0;
        } else {
            output = dp*alpha_2 + f0; 
        }
        return output;
    };
};


class SmoothedDerivative3D {
public:
    SmoothedDerivative3D(size_t window_size, float sigma, float dt)
        : window_size_(window_size), dt_(dt),
          kernel_(compute_derivative_gaussian_kernel(window_size, sigma, dt)),
          buffers_(3, std::vector<float>(window_size, 0.0f)),
          index_(0)
    {}

    // Add new sample and get smoothed derivative
    std::vector<float> filter(float gx, float gy, float gz) {
        // Write sample into circular buffer
        size_t write_index = index_.fetch_add(1, std::memory_order_relaxed) % window_size_;
        buffers_[0][write_index] = gx;
        buffers_[1][write_index] = gy;
        buffers_[2][write_index] = gz;

        // Compute result
        std::vector<float> result(3, 0.0f);
        for (int axis = 0; axis < 3; ++axis) {
            for (size_t k = 0; k < window_size_; ++k) {
                size_t buf_index = (write_index + window_size_ - k) % window_size_;
                result[axis] += buffers_[axis][buf_index] * kernel_[k];
            }
        }
        return result;
    }

private:
    size_t window_size_;
    float dt_;
    std::vector<float> kernel_;
    std::vector<std::vector<float>> buffers_;  // 3 × circular buffers
    std::atomic<size_t> index_;

    // Compute causal derivative-of-Gaussian kernel
    std::vector<float> compute_derivative_gaussian_kernel(size_t N, float sigma, float dt) {
        std::vector<float> kernel(N);

        for (size_t i = 0; i < N; ++i) {
            float t = -static_cast<float>(i) * dt;
            // derivative of Gaussian 
            kernel[i] = -(t / (sigma * sigma)) * std::exp(-t * t / (2.0f * sigma * sigma));
        }

        // 1. Zero mean 
        float mean = std::accumulate(kernel.begin(), kernel.end(), 0.0f) / static_cast<float>(N);
        for (auto &v : kernel) v -= mean;

        // 2. Normalize energy 
        float energy = std::sqrt(std::inner_product(kernel.begin(), kernel.end(), kernel.begin(), 0.0f));
        if (energy > 0.0f)
            for (auto &v : kernel) v /= energy;

        return kernel;
    }
};

// Small helpers to convert between ros timestamps and milliseconds

inline void ms_to_ros_stamp(const double ms, ros::Time &stamp) {
    stamp.sec = static_cast<int>(std::floor(ms / 1000.0));
    double ms_remainder = ms - stamp.sec * 1000.0;
    stamp.nsec = static_cast<int>(ms_remainder * 1e6);
}

inline double ros_stamp_to_ms(const ros::Time &stamp) {
    return static_cast<double>(stamp.sec) * 1000.0 +
    static_cast<double>(stamp.nsec) / 1e6;
}

#endif /* IMU_JASPER_UTILS_H */