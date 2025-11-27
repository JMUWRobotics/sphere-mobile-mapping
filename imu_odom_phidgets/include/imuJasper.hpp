/**
 * @author Fabian Arzberger, JMU
 * @author Jasper Zevering, JMU 
 * 
 * This header defines the structural framework where multiple IMU measurements are 
 * merged into consistent accelerometer and gyroscope readings. 
 * 
 */
#ifndef IMU_MERGER_H
#define IMU_MERGER_H

// System
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <mutex>
#include <atomic>
#include <math.h>
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

// Filter definitions and utils from this project
#include "utils.hpp"
#include "filter.hpp"
#if defined(MAHONY)
    #include "mahony.hpp"
#elif defined(QEKF) 
    #include "ekf.hpp"
#elif defined(UKF)
    #include "ukf.hpp"
#else
    #include "autogain.hpp"
#endif

#define DATA_RATE_DEFAULT 250
#define IMU_DATA_RATE_DEFAULT 250

#define DEFAULT_MADGWICK_GAIN 0.1
#define ALPHA_COMPLEMENTARY_DEFAULT 0.2

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

// Each IMU needs its own time synchronization
passive_time_sync sync0, sync1, sync2;

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

/**
 * -------- START INTERNAL STATE OF THE PROGRAM ---------
 *
 * Note that this is quite ugly since all the state variables are just 
 * defined here on the stack not belonging to any object.
 * 
 */
// name of topic
std::string topicName = "posePub_merged";
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
// position
float px;
float py;
float pz;
#if defined(MAHONY)
    // mahony PI gains
    double Kp, Ki;
#elif defined(QEKF) || defined(UKF)
    // Kalman filter covariances
    float sigma_gyr, sigma_a, P_init;
#else
    // gain of madgwickfilter
    float gain_ = DEFAULT_MADGWICK_GAIN; 
    float gain_min;
    // gain of complemnetary
    float alpha = ALPHA_COMPLEMENTARY_DEFAULT;
    // autogain value
    float autogain = 0;
#endif 
#if defined(UKF)
    float alpha, beta, kappa;
#endif
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
// accelerometer
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
// velocity 
float vel_x;
float vel_y;
float vel_z;
// delta time between iterations
float dt; 
// timestamps (in ms)
double lastTime;
double lastTime_serial0_ms;
double lastTime_serial1_ms;
double lastTime_serial2_ms;
int firstRead = 0;
float r = 0; // radius of sphere 
// n1 counts the number of samples gathered before the mean is taken of it (if) publishing rate is lower then imu rate
int n0 = 0;
int n1 = 0;
int n2 = 0;
std::mutex mtx_n0, mtx_n1, mtx_n2; // protects the averages
int data_rate = DATA_RATE_DEFAULT;
float data_intervall = 1 / DATA_RATE_DEFAULT;
int imu_data_rate = IMU_DATA_RATE_DEFAULT;
// determines if z should always be 0 (helps improving position if this assumption can be made))
bool setZ0 = false;
// If true, publish motion force compensated accelerometer values
bool debugMode = false;
// slow-mode activates moving average of gyroscope (g_new = (g_old +g_0+ g_1 + g_2)/4) 
bool slow = false;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

/**
 * -------- END INTERNAL STATE OF THE PROGRAM ---------
 */


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

/* Calculates position of the sphere based on rotation speed */
inline void calc_position(float gx, float gy, float gz);

/* The important magic function, calls the filter function of the AttitudeEstimator instance */
void apply_attitude_filter(double stamp_ms, bool reuse_dt = false);

// Filter pointer to the AttitudeEstimator
AttitudeEstimator* estimator_instance;

// Derivative of Gaussian kernel pointer
SmoothedDerivative3D *smooth_deriv_kernel;

// Data assosciacion and centripetal compensation
void setValsAndCompensateCentripetal(const int serialNr,
    const double acceleration[3], const double angularRate[3], const double dts,
    float *gx, float *gy, float *gz, float *ax, float *ay, float *az
); 

//for safe shutdown
void mySigIntHandler(int sig);

//ovewrite q0-q3 with an estimation of accelerometer and the desired yaw (can not be set from accelerometer
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw);

#endif /* IMU_MERGER_H */