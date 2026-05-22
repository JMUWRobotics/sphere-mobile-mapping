/**
 * @author Fabian Arzberger, JMU
 * @author Jasper Zevering, JMU
 *
 * filter.hpp
 *
 * Interface and shared state for the attitude-estimation / fusion node
 * (filter.cpp).  This header:
 *   - Retains the AttitudeEstimator abstract base class.
 *   - Declares the filter-node state (quaternion, position, gyro, accel,
 *     centripetal buffers, alternative evaluation state, …).
 *   - Declares all functions implemented in filter.cpp.
 */
#ifndef FILTER_INTERFACE_H
#define FILTER_INTERFACE_H

// System
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <cmath>
#include <mutex>
#include <numeric>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom estimator msg
#include <state_estimator_msgs/Estimator.h>

// Ground normal msg
#include <ground_finder_msgs/ScoredNormalStamped.h>

// Shared utilities
#include "utils.hpp"

// -----------------------------------------------------------------------
// Constants (mirrored from imuJasper.hpp to keep self-contained)
// -----------------------------------------------------------------------
#ifndef DATA_RATE_DEFAULT
#define DATA_RATE_DEFAULT 250
#endif
#ifndef IMU_DATA_RATE_DEFAULT
#define IMU_DATA_RATE_DEFAULT 250
#endif

#define DEFAULT_MADGWICK_GAIN 0.1
#define ALPHA_COMPLEMENTARY_DEFAULT 0.2

#ifndef M_PI_BY_180
#define M_PI_BY_180 0.017453292
#endif
#ifndef precalc_180_BY_M_PI
#define precalc_180_BY_M_PI 57.29577951
#endif

// -----------------------------------------------------------------------
// AttitudeEstimator — abstract interface
// All concrete filters (Mahony, AutoGain, QEKF, UKF) derive from this.
// -----------------------------------------------------------------------
class AttitudeEstimator
{
protected:
    float q0, q1, q2, q3; /* internal quaternion state */

public:
    virtual quaternion filter(const Vec3 &gyr, const Vec3 &acc, float dt) = 0;

    /**
     * Force the internal quaternion state to a known value.
     * Use only during initialisation or explicit re-initialisation.
     */
    void override_state(float qw, float qx, float qy, float qz)
    {
        q0 = qw;
        q1 = qx;
        q2 = qy;
        q3 = qz;
        normalizeQuat(&q0, &q1, &q2, &q3);
    }
};

// Concrete filter implementations selected at compile time
#if defined(MAHONY)
#include "mahony.hpp"
#elif defined(QEKF)
#include "ekf.hpp"
#elif defined(UKF)
#include "ukf.hpp"
#else
#include "autogain.hpp"
#endif

// -----------------------------------------------------------------------
// Filter-node global state
// -----------------------------------------------------------------------

// Output topic name (configurable via ROS param "topicName")
extern std::string topicName;

// --- Quaternion of current estimate state ---
extern float q0, q1, q2, q3;
extern float lastq0, lastq1, lastq2, lastq3;

// --- Position ---
extern float px, py, pz;
extern float r; // sphere radius

// --- Merged gyroscope values (rad/s) ---
extern float gx, gy, gz;
extern float lastgx, lastgy, lastgz;
// Per-IMU gyroscope values (populated by ROS callbacks in filter.cpp)
extern float gx0, gy0, gz0;
extern float gx1, gy1, gz1;
extern float gx2, gy2, gz2;
// Quaternion-filtered rotation speed
extern float gx_filtered, gy_filtered, gz_filtered;

// --- Merged accelerometer values (g, centripetal-compensated) ---
extern float ax, ay, az;
// Per-IMU centripetal-compensated accumulators
extern float ax0, ay0, az0;
extern float ax1, ay1, az1;
extern float ax2, ay2, az2;

// --- Velocity ---
extern float vel_x, vel_y, vel_z;

// --- Timing ---
extern float dt;
extern double lastTime; // ms, tracks merged stamp for dt computation

// Per-IMU last received stamp (ms); set by the subscriber callbacks
extern double lastTime_serial0_ms;
extern double lastTime_serial1_ms;
extern double lastTime_serial2_ms;

extern int firstRead;
extern int n_imus;
extern bool use_serial0, use_serial1, use_serial2;

// --- Averaging sample counters ---
extern int n0, n1, n2;
extern std::mutex mtx_n0, mtx_n1, mtx_n2;

// --- Centripetal compensation state ---
// Extrinsic lever-arm vectors (position of each IMU wrt sphere centre, in IMU frame)
extern float *pos_serial0;
extern float *pos_serial1;
extern float *pos_serial2;

// Serial numbers
extern int SERIAL_0, SERIAL_1, SERIAL_2;

// Uncompensated accel accumulators (used by evaluation)
extern float a0_alt[3], a1_alt[3], a2_alt[3];
extern float ax_alt, ay_alt, az_alt;

// Compensated accel (only populated in debugMode)
extern float a0_comp[3], a1_comp[3], a2_comp[3];

// misc flags
extern bool slow;      // moving-average of gyro
extern bool setZ0;     // force pz = 0
extern bool debugMode; // publish compensated vs raw comparison

// Threshold on squared angular speed for position integration
extern float small_omega2;

// --- Filter-specific gains / covariances ---
#if defined(MAHONY)
extern double Kp, Ki;
#elif defined(QEKF) || defined(UKF)
extern float sigma_gyr, sigma_a, P_init;
#else
extern float gain_;
extern float gain_min;
extern float alpha;
extern float autogain;
#endif
#if defined(UKF)
extern float alpha, beta, kappa;
#endif

// Signal-safe shutdown flag
extern sig_atomic_t volatile g_request_shutdown;

// --- Alternative evaluation state ---
extern float q0_alt1, q1_alt1, q2_alt1, q3_alt1;
extern float q0_alt2, q1_alt2, q2_alt2, q3_alt2;
extern float q0_last_alt1, q1_last_alt1, q2_last_alt1, q3_last_alt1;
extern float q0_last_alt2, q1_last_alt2, q2_last_alt2, q3_last_alt2;
extern int altStatus;
extern float pXAlt, pYAlt;

// Pointer to active attitude estimator.
// Three independent smoothed-derivative kernels(one per IMU) so their internal ring-buffer histories never mix when callbacks interleave
extern AttitudeEstimator *estimator_instance;
extern SmoothedDerivative3D *smooth_deriv_kernel0;
extern SmoothedDerivative3D *smooth_deriv_kernel1;
extern SmoothedDerivative3D *smooth_deriv_kernel2;

// --- Ground normal stuff ---
extern bool use_ground_normal;
extern std::string ground_normal_topic;
extern float gn_x, gn_y, gn_z; // ground normal vector (default: [0, 0, -1])
extern std::mutex mtx_gn;
extern bool ground_normal_available;

// --- TF2 buffer for frame transformations ---
extern tf2_ros::Buffer tf_buffer_;
extern std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
extern ros::Subscriber ground_normal_sub;

// -----------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------

/** Parse filter-node ROS parameters. */
int argumentHandler(ros::NodeHandle &nh);

/**
 * Per-IMU centripetal compensation + accumulation into ax/ay/az and gx/gy/gz.
 * Must be called for each IMU before combineRAWData().
 * @param kernel  The per-IMU smoothed-derivative filter (one per IMU).
 */
void setValsAndCompensateCentripetal(
    int serialNr,
    const double acceleration[3],
    const double angularRate[3],
    SmoothedDerivative3D *kernel,
    float *gx, float *gy, float *gz,
    float *ax, float *ay, float *az);

/**
 * Average the per-IMU compensated measurements into a single gx/gy/gz and
 * ax/ay/az.  Pass false for IMUs that should be excluded from the accel
 * average
 */
int combineRAWData(bool use0 = true, bool use1 = true, bool use2 = true);

/** Run one step of the attitude estimator and update q0...q3. */
void apply_attitude_filter(double stamp_ms, bool reuse_dt = false);

/** Integrate position from filtered rotation speed and accelerometer. */
void calc_position(float gx, float gy, float gz);

/**
 * Swap between primary and alternative quaternion state for evaluation.
 * Lets the main loop run the filter twice per cycle (once with compensated
 * data, once without) and compare results.
 */
void changeAlt();

/** Overwrite q0...q3 with an estimate derived from the accelerometer + a desired yaw. */
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw);

/** Replacement SIGINT handler. */
void mySigIntHandler(int sig);

#endif /* FILTER_INTERFACE_H */
