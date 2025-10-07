#ifndef IMU_MERGER_H
#define IMU_MERGER_H

// System
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <mutex>
#include <atomic>
#include <cmath>
#include <numeric>

// Phidget
#include <phidget22.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Custom msg
#include <state_estimator_msgs/Estimator.h>

// YAML to load calibration matrices
#include <yaml-cpp/yaml.h>

#define DATA_RATE_DEFAULT 250
#define IMU_DATA_RATE_DEFAULT 250

#define DEFAULT_MADGWICK_GAIN 0.1

//often used calculations
#define M_PI_BY_180 0.017453292
#define precalc_180_BY_M_PI 57.29577951
#define ONE_THIRD 0.333333333

// NON-CONST Serial numbers of the IMUs
int SERIAL_0, SERIAL_1, SERIAL_2;

// Intrinsic of each IMU
std::vector<double> acc0_misalignment, acc0_scale, acc0_bias;
std::vector<double> acc1_misalignment, acc1_scale, acc1_bias;
std::vector<double> acc2_misalignment, acc2_scale, acc2_bias;
std::vector<double> gyr0_misalignment, gyr0_scale, gyr0_bias;
std::vector<double> gyr1_misalignment, gyr1_scale, gyr1_bias;
std::vector<double> gyr2_misalignment, gyr2_scale, gyr2_bias;
double *acc0_calibrated;
double *acc1_calibrated;
double *acc2_calibrated;
double *gyr0_calibrated;
double *gyr1_calibrated;
double *gyr2_calibrated;

// Extrinsics of each IMU wrt the center point of the sphere
float *pos_serial0;
float *pos_serial1;
float *pos_serial2;

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

passive_time_sync sync0, sync1, sync2;
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
    }
    quaternion(double q0, double q1, double q2, double q3) : w(q0), x(q1), y(q2), z(q3) {
    }
};

//quaternion operations

// Spherical linear interpolation, input l and r, output o, weight [0-1] between l and r
void quaternion_slerp(quaternion *l, quaternion *r, quaternion *o, double weight);
// Simple quaternion dot product (needed inside slerp!) DO NOT confuse this with quaternion multiplication! 
double quaternion_dot_product(quaternion *l, quaternion *r);
// Normalizes the quaternion (alters the input!)
void quaternion_normalize(quaternion *q);
// Copy quaternion
void quaternion_copy(quaternion *original_q, quaternion *copy_q);
// Invert quaternion q, put result in res
void quaternion_inverse(const quaternion* q, quaternion* res);
// Calculate res = q1 * q2
void quaternion_multiply(const quaternion* q1, const quaternion* q2, quaternion* res);
// Convert quaternion to axis-angle represenation 
void quaternion_to_axis_angle(const quaternion* in, double* axis, double* angle);

extern bool quiet;
//actual data rate of publishing. will be set to DATA_RATE_DEFAULT if not else stated
extern int data_rate;

// ROS messages to be published
sensor_msgs::Imu imu0_msg, imu1_msg, imu2_msg;
ros::Publisher imu0_raw_pub, imu1_raw_pub, imu2_raw_pub;
ros::Publisher imu0_calib_pub, imu1_calib_pub, imu2_calib_pub;
state_estimator_msgs::Estimator output_msg;
int seq, seq0, seq1, seq2; // sequence number that counts number of msgs

// ROS subscribe to master clock that we want to (passively) sync to
ros::Subscriber clock_sub;

//orientation
float q0; 
float q1; 
float q2; 
float q3; 
// last iteration
float lastq0;
float lastq1;
float lastq2;
float lastq3;
float px;
float py;
float pz;
//gain of madgwickfilter
extern float gain_; 
float gain_min;
//gain of complemnetary
extern float alpha;
//gyroscope
float gx; 
float gy; 
float gz; 
float lastgx = 0;
float lastgy = 0;
float lastgz = 0;
float gx1;
float gy1;
float gz1;
float gx2;
float gy2;
float gz2;
float gx0;
float gy0;
float gz0;
// filtered gyroscope (based on relative quaternion orientation)
float gx_filtered;
float gy_filtered;
float gz_filtered;
//accelerometer
float ax; 
float ay; 
float az;
float ax1;
float ay1;
float az1;
float ax2;
float ay2;
float az2;
float ax0;
float ay0;
float az0; 

float vel_x;
float vel_y;
float vel_z;

float dt; 

extern int firstRead;

extern bool slow;

double lastTime;
double lastTime_serial0_ms;
double lastTime_serial1_ms;
double lastTime_serial2_ms;
float lastw;

//Functions
//initalizing procedure
int CCONV init();
//handlign arguments giving to programm
int argumentHandler(ros::NodeHandle &nh);

// apply intrinsic calibration parameters
void apply_intrinsics(const double raw[3], 
    std::vector<double> &align,
    std::vector<double> &scale,
    std::vector<double> &bias,
    double* result
);

// Put phidget callback message into corresponding ros msg
inline void process_phidget_to_calibrated_ros_msg(
    int serial, int spatial, 
    double *acc_inverse, double* angular_radians, 
    float* gx, float* gy, float* gz,
    float* ax, float* ay, float* az,
    passive_time_sync &sync,
    double ros_now_s, double timestamp, double &last_time_ms, 
    std::vector<double> &gyr_align,
    std::vector<double> &gyr_scale,
    std::vector<double> &gyr_bias,
    std::vector<double> &acc_align,
    std::vector<double> &acc_scale,
    std::vector<double> &acc_bias,
    double *acc_calibrated, double* gyr_calibrated,
    const char *frame,
    int &count,
    sensor_msgs::Imu &imu_msg,
    ros::Publisher &raw_pub,
    ros::Publisher &calibrated_pub 
);

//main filter function. @param reuse_dt: Set true if filter should be applied with alternative input values
void madgwick_and_complementary_Filter(double stamp_ms, bool reuse_dt = false);

// Data assosciacion and centripetal compensation
void setValsAndCompensateCentripetal(const int serialNr,
    const double acceleration[3], const double angularRate[3], const double dts,
    float *gx, float *gy, float *gz, float *ax, float *ay, float *az
); 

//for safe shutdown
void mySigIntHandler(int sig);
// calculate qauternion an write it into the qW to qZ
void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw);
//calculate euler an write it into yaw pitch roll
void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ);
//normalize quaternion values inplace
inline void normalizeQuat(float *q0, float *q1, float *q2, float *q3);
inline void vectorAsNormalized(const float* v, float* res);
inline void vectorCross(const float* v1, const float* v2, float* res);
//ovewrite q0-q3 with an estimation of accelerometer and the desired yaw (can not be set from accelerometer
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw);

inline void ms_to_ros_stamp(const double ms, ros::Time &stamp) {
    stamp.sec = static_cast<int>(std::floor(ms / 1000.0));
    double ms_remainder = ms - stamp.sec * 1000.0;
    stamp.nsec = static_cast<int>(ms_remainder * 1e6);
}

inline double ros_stamp_to_ms(const ros::Time &stamp) {
    return static_cast<double>(stamp.sec) * 1000.0 +
    static_cast<double>(stamp.nsec) / 1e6;
}

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

#endif
