/**
 * @author Fabian Arzberger, JMU
 * @author Jasper Zevering, JMU
 *
 * Sensor-acquisition node header.
 * Responsibilities:
 *   - Open Phidget IMUs
 *   - Apply intrinsic calibration (bias, scale, misalignment)
 *   - Timestamp synchronisation
 *   - Publish per-IMU /imu/N/raw and /imu/N/calib topics
 *
 * Everything related to attitude estimation, centripetal compensation,
 * multi-IMU averaging, and pose publishing lives in filter.cpp / filter.hpp.
 */
#ifndef IMU_JASPER_H
#define IMU_JASPER_H

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
#include <sensor_msgs/Imu.h>

// YAML to load calibration matrices
#include <yaml-cpp/yaml.h>

// Shared utilities (passive_time_sync, ms_to_ros_stamp, …)
#include "utils.hpp"

// Compile-time constants
#define DATA_RATE_DEFAULT 250
#define IMU_DATA_RATE_DEFAULT 250

#define M_PI_BY_180 0.017453292
#define precalc_180_BY_M_PI 57.29577951

// NON-CONST Serial numbers of the IMUs
int SERIAL_0, SERIAL_1, SERIAL_2;
bool use_serial0, use_serial1, use_serial2;
bool initialized0, initialized1, initialized2;
int n_imus; // counts how many IMUs successfully checked in at runtime

// Intrinsic of each IMU (loaded from yaml)
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

// Each IMU needs its own time synchronization
passive_time_sync sync0, sync1, sync2;

extern bool quiet;

// actual data rate of publishing. will be set to DATA_RATE_DEFAULT if not else stated
int data_rate = DATA_RATE_DEFAULT;
float data_intervall = 1.0f / DATA_RATE_DEFAULT;
int imu_data_rate = IMU_DATA_RATE_DEFAULT;

// In debugMode the calibrated topic carries compensated accel values
// (compensation still done in filter.cpp, but flag is read here so the
//  raw topic can swap its payload accordingly — see process_phidget_to_calibrated_ros_msg).
bool debugMode = false;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// ROS publishers and messages
sensor_msgs::Imu imu0_msg, imu1_msg, imu2_msg;

ros::Publisher imu0_raw_pub, imu1_raw_pub, imu2_raw_pub;
ros::Publisher imu0_calib_pub, imu1_calib_pub, imu2_calib_pub;

int seq0 = 0, seq1 = 0, seq2 = 0; // per-IMU message sequence counters

// Per-IMU last-callback timestamp (ms); used to compute dt inside callback
double lastTime_serial0_ms = 0.0;
double lastTime_serial1_ms = 0.0;
double lastTime_serial2_ms = 0.0;

// -----------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------

/** Parse ROS parameters and load YAML calibration files. */
int argumentHandler(ros::NodeHandle &nh);

/** Initialise Phidget handles, open devices, zero gyros. */
int CCONV init();

// apply intrinsic calibration parameters
void apply_intrinsics(
    const double raw[3],
    std::vector<double> &align,
    std::vector<double> &scale,
    std::vector<double> &bias,
    double *result);

// Put phidget callback message into corresponding ros msg
void process_phidget_to_calibrated_ros_msg(
    int serial, int spatial,
    double *acc_inverse, double *angular_radians,
    passive_time_sync &sync,
    double ros_now_s, double timestamp, double &last_time_ms,
    std::vector<double> &gyr_align,
    std::vector<double> &gyr_scale,
    std::vector<double> &gyr_bias,
    std::vector<double> &acc_align,
    std::vector<double> &acc_scale,
    std::vector<double> &acc_bias,
    double *acc_calibrated, double *gyr_calibrated,
    const char *frame,
    int &count,
    sensor_msgs::Imu &imu_msg,
    ros::Publisher &raw_pub,
    ros::Publisher &calibrated_pub);

// for safe shutdown
void mySigIntHandler(int sig);

#endif /* IMU_JASPER_H */
