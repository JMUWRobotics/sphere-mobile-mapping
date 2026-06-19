/**
 * @author Fabian Arzberger, JMU
 * @author Jasper Zevering, JMU
 *
 * filter.cpp  —  Attitude estimation and pose fusion node.
 *
 * This node:
 *   1. Subscribes to the three calibrated IMU topics published by imu_jasper:
 *        /imu/0/calib   /imu/1/calib   /imu/2/calib
 *   2. For each incoming message applies per IMU centripetal
 *      compensation (needs the extrinsic position vectors from the param server).
 *   3. Averages the compensated measurements across active IMUs.
 *   4. Runs the compile-time selected attitude estimator
 *      (Mahony | AutoGain | QEKF | UKF).
 *   5. Integrates position from filtered rotation speed and accel.
 *   6. Publishes the result as state_estimator_msgs/Estimator on the same
 *      output topic that the monolithic imuJasper node used to publish on,
 *      so all downstream nodes are unaffected.
 *
 * Bag-file replay works naturally: just replay the /imu/N/calib topics and
 * this node will reproduce the same estimates from the recorded data.
 */
#include "filter.hpp"
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// -----------------------------------------------------------------------
// Global state definitions  (declared extern in filter.hpp)
// -----------------------------------------------------------------------
std::string topicName = "posePub_merged";

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float lastq0 = 1, lastq1 = 0, lastq2 = 0, lastq3 = 0;

float px = 0, py = 0, pz = 0;
float r = 0;

float gx = 0, gy = 0, gz = 0;
float lastgx = 0, lastgy = 0, lastgz = 0;
float gx0 = 0, gy0 = 0, gz0 = 0;
float gx1 = 0, gy1 = 0, gz1 = 0;
float gx2 = 0, gy2 = 0, gz2 = 0;
float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;

float ax = 0, ay = 0, az = 0;
float ax0 = 0, ay0 = 0, az0 = 0;
float ax1 = 0, ay1 = 0, az1 = 0;
float ax2 = 0, ay2 = 0, az2 = 0;

float vel_x = 0, vel_y = 0, vel_z = 0;

float dt = 0;
double lastTime = 0;

double lastTime_serial0_ms = 0;
double lastTime_serial1_ms = 0;
double lastTime_serial2_ms = 0;

int firstRead = 0;
int n_imus = 0;
bool use_serial0 = false, use_serial1 = false, use_serial2 = false;

int n0 = 0, n1 = 0, n2 = 0;
std::mutex mtx_n0, mtx_n1, mtx_n2;

float *pos_serial0 = nullptr;
float *pos_serial1 = nullptr;
float *pos_serial2 = nullptr;

int SERIAL_0 = 0, SERIAL_1 = 0, SERIAL_2 = 0;

float a0_alt[3] = {0, 0, 0};
float a1_alt[3] = {0, 0, 0};
float a2_alt[3] = {0, 0, 0};
float ax_alt = 0, ay_alt = 0, az_alt = 0;
float a0_comp[3] = {0, 0, 0};
float a1_comp[3] = {0, 0, 0};
float a2_comp[3] = {0, 0, 0};

bool slow = false;
bool setZ0 = false;
bool debugMode = false;

float small_omega2 = 0.01f;

#if defined(MAHONY)
double Kp = 1.0, Ki = 0.001;
#elif defined(QEKF) || defined(UKF)
float sigma_gyr = 0.001f, sigma_a = 0.001f, P_init = 0.01f;
#else
float gain_ = DEFAULT_MADGWICK_GAIN;
float gain_min = DEFAULT_MADGWICK_GAIN;
float alpha = ALPHA_COMPLEMENTARY_DEFAULT;
float autogain = 0;
#endif
#if defined(UKF)
float alpha = 0.001f, beta = 2.0f, kappa = 0.0f;
#endif

sig_atomic_t volatile g_request_shutdown = 0;

// Alternative evaluation state
float q0_alt1 = 1, q1_alt1 = 0, q2_alt1 = 0, q3_alt1 = 0;
float q0_alt2 = 1, q1_alt2 = 0, q2_alt2 = 0, q3_alt2 = 0;
float q0_last_alt1 = 1, q1_last_alt1 = 0, q2_last_alt1 = 0, q3_last_alt1 = 0;
float q0_last_alt2 = 1, q1_last_alt2 = 0, q2_last_alt2 = 0, q3_last_alt2 = 0;
int altStatus = 1;
float pXAlt = 0, pYAlt = 0, pZAlt = 0;

AttitudeEstimator *estimator_instance = nullptr;
// One derivative kernel per IMU: each kernel holds its own ring-buffer
// state, so interleaving calls from different IMU callbacks is safe.
SmoothedDerivative3D *smooth_deriv_kernel0 = nullptr;
SmoothedDerivative3D *smooth_deriv_kernel1 = nullptr;
SmoothedDerivative3D *smooth_deriv_kernel2 = nullptr;
SmoothedDerivative3D *smooth_deriv_kernelN = nullptr;

// Latest header stamp received from each IMU (seconds).
// Written by the subscriber callbacks; read by the main loop to compute
// the averaged combined timestamp used for dt calculation.
// Initialised to 0 so the main loop can detect "no message yet".
static double last_stamp0_s = 0.0;
static double last_stamp1_s = 0.0;
static double last_stamp2_s = 0.0;

// Set to true the first time a message arrives on each topic.
// Used as the startup-readiness gate instead of checking gx == 0,
// which would wrongly block on a legitimate zero gyro reading.
static bool imu0_ready = false;
static bool imu1_ready = false;
static bool imu2_ready = false;

// ROS output
static int seq = 0;
static state_estimator_msgs::Estimator output_msg;

// Ground Normal Variables
bool use_ground_normal = false;
std::string ground_normal_topic = "ground_normal";
float gn_x = 0.0f;
float gn_y = 0.0f;
float gn_z = -1.0f; // default: downward normal
std::mutex mtx_gn;  // protects ground normal vector
bool ground_normal_available = false;

// Ground Normal for Centripetal Compensation (in IMU frame)
bool use_normal_centripetal = false;
float gn_x_imu = 0.0f;
float gn_y_imu = 0.0f;
float gn_z_imu = -1.0f; // default: downward normal
float gn_x_imu_prev = 0.0f;
float gn_y_imu_prev = 0.0f;
float gn_z_imu_prev = -1.0f; // previous normal for derivative
std::mutex mtx_gn_imu;
bool ground_normal_centripetal_available = false;

// --- TF2 buffer for frame transformations ---
tf2_ros::Buffer tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
ros::Subscriber ground_normal_sub;
ros::Subscriber ground_normal_centripetal_sub;

// Static TF used to rotate ground normals from source sensor frame into IMU frame.
static bool ground_normal_tf_ready = false;
static geometry_msgs::TransformStamped ground_normal_tf;
static std::string ground_normal_source_frame = "pandar_frame";
static std::string ground_normal_target_frame = "imu_frame";

// -----------------------------------------------------------------------
// changeAlt  —  swap between primary and alternative quaternion state
// -----------------------------------------------------------------------
void changeAlt()
{
    if (altStatus == 1)
    {
        altStatus = 2;
        q0_alt1 = q0;
        q1_alt1 = q1;
        q2_alt1 = q2;
        q3_alt1 = q3;
        q0_last_alt1 = lastq0;
        q1_last_alt1 = lastq1;
        q2_last_alt1 = lastq2;
        q3_last_alt1 = lastq3;
        q0 = q0_alt2;
        q1 = q1_alt2;
        q2 = q2_alt2;
        q3 = q3_alt2;
        lastq0 = q0_last_alt2;
        lastq1 = q1_last_alt2;
        lastq2 = q2_last_alt2;
        lastq3 = q3_last_alt2;
    }
    else
    {
        altStatus = 1;
        q0_alt2 = q0;
        q1_alt2 = q1;
        q2_alt2 = q2;
        q3_alt2 = q3;
        q0_last_alt2 = lastq0;
        q1_last_alt2 = lastq1;
        q2_last_alt2 = lastq2;
        q3_last_alt2 = lastq3;
        q0 = q0_alt1;
        q1 = q1_alt1;
        q2 = q2_alt1;
        q3 = q3_alt1;
        lastq0 = q0_last_alt1;
        lastq1 = q1_last_alt1;
        lastq2 = q2_last_alt1;
        lastq3 = q3_last_alt1;
    }
    estimator_instance->override_state(q0, q1, q2, q3);
}

// -----------------------------------------------------------------------
// setValsAndCompensateCentripetal
//
// Apply the lever-arm / centripetal correction for one IMU and accumulate
// the result into the per-IMU running totals (ax0/ax1/ax2, gx0/gx1/gx2).
// The plain calibrated acceleration (no compensation) is also accumulated
// in a0_alt for the evaluation path.
//
// Two models supported (switchable via use_normal_centripetal flag):
//   Eq. 5 (flat-ground):  a_g = w × (w × r) + (dw/dt) × r
//   Eq. 12 (with normal): a_g = w × (w × r_e) + (dw/dt) × r_e + R*(dw/dt) × n + R*w × (dn/dt)
// -----------------------------------------------------------------------
void setValsAndCompensateCentripetal(
    int serialNr,
    const double acceleration[3],
    const double angularRate[3],
    SmoothedDerivative3D *kernel,
    float *gx_out, float *gy_out, float *gz_out,
    float *ax_out, float *ay_out, float *az_out)
{
    float cur_gx = static_cast<float>(angularRate[0]);
    float cur_gy = static_cast<float>(angularRate[1]);
    float cur_gz = static_cast<float>(angularRate[2]);

    // Smoothed angular-rate derivative (tangential term) — uses this IMU's kernel
    auto gdot = kernel->filter(cur_gx, cur_gy, cur_gz);
    float wdot[3] = {gdot[0], gdot[1], gdot[2]};

    // Identify lever-arm for this IMU
    float *pos_serial = nullptr;
    if (serialNr == SERIAL_0)
    {
        a0_alt[0] += acceleration[0];
        a0_alt[1] += acceleration[1];
        a0_alt[2] += acceleration[2];
        pos_serial = pos_serial0;
    }
    else if (serialNr == SERIAL_1)
    {
        a1_alt[0] += acceleration[0];
        a1_alt[1] += acceleration[1];
        a1_alt[2] += acceleration[2];
        pos_serial = pos_serial1;
    }
    else if (serialNr == SERIAL_2)
    {
        a2_alt[0] += acceleration[0];
        a2_alt[1] += acceleration[1];
        a2_alt[2] += acceleration[2];
        pos_serial = pos_serial2;
    }
    else
    {
        ROS_ERROR("setValsAndCompensateCentripetal: unknown serial %d", serialNr);
        return;
    }

    float axval, ayval, azval;

    if (!use_normal_centripetal)
    {
        // ========== EQUATION 5: Flat-ground model ==========
        // ω × r  (centripetal direction)
        float gyr[3] = {cur_gx, cur_gy, cur_gz};
        float gyr_cross_r[3];
        vectorCross(gyr, pos_serial, gyr_cross_r);

        // ω × (ω × r)  (centripetal acceleration)
        float gyr_cross_gyr_cross_r[3];
        vectorCross(gyr, gyr_cross_r, gyr_cross_gyr_cross_r);

        // ω̇ × r  (tangential acceleration due to angular acceleration)
        float wdot_cross_r[3];
        vectorCross(wdot, pos_serial, wdot_cross_r);

        axval = acceleration[0] - gyr_cross_gyr_cross_r[0] - wdot_cross_r[0];
        ayval = acceleration[1] - gyr_cross_gyr_cross_r[1] - wdot_cross_r[1];
        azval = acceleration[2] - gyr_cross_gyr_cross_r[2] - wdot_cross_r[2];
    }
    else
    {
        // ========== EQUATION 12: Normal-based model ==========
        // Get current and previous normal vectors (in IMU frame)
        float n_imu[3], n_imu_prev[3];
        mtx_gn_imu.lock();
        n_imu[0] = gn_x_imu;
        n_imu[1] = gn_y_imu;
        n_imu[2] = gn_z_imu;
        n_imu_prev[0] = gn_x_imu_prev;
        n_imu_prev[1] = gn_y_imu_prev;
        n_imu_prev[2] = gn_z_imu_prev;
        mtx_gn_imu.unlock();

        auto ndot = smooth_deriv_kernelN->filter(gn_x_imu, gn_y_imu, gn_z_imu);

        // Compute normal rate-of-change: dn/dt
        float dn_dt[3];
        dn_dt[0] = ndot[0];
        dn_dt[1] = ndot[1];
        dn_dt[2] = ndot[2];

        // ω × r_e
        float gyr[3] = {cur_gx, cur_gy, cur_gz};
        float gyr_cross_r_e[3];
        vectorCross(gyr, pos_serial, gyr_cross_r_e); // double check if pos_serial is really r_e

        // ω × (ω × r_e)  (centripetal acceleration, same as before)
        float gyr_cross_gyr_cross_r_e[3];
        vectorCross(gyr, gyr_cross_r_e, gyr_cross_gyr_cross_r_e);

        // ω̇ × r_e  (tangential acceleration)
        float wdot_cross_r_e[3];
        vectorCross(wdot, pos_serial, wdot_cross_r_e);

        // R * (ω̇ × n)  sphere radius times normal dependent angular acceleration
        float R_wdot_cross_n[3];
        float R_wdot[3] = {r * wdot[0], r * wdot[1], r * wdot[2]};
        vectorCross(R_wdot, n_imu, R_wdot_cross_n);

        // R * (ω × dn/dt)  sphere radius times normal-rate coupling
        float gyr_cross_dn_dt[3];
        vectorCross(gyr, dn_dt, gyr_cross_dn_dt);
        float R_gyr_cross_dn_dt[3] = {r * gyr_cross_dn_dt[0], r * gyr_cross_dn_dt[1], r * gyr_cross_dn_dt[2]};

        // Combine all terms to eq 12: a_g = w×(w×r_e) + (dw/dt)×r_e + R*(dw/dt)×n + R*w×(dn/dt)
        axval = acceleration[0] - gyr_cross_gyr_cross_r_e[0] - wdot_cross_r_e[0] - R_wdot_cross_n[0] - R_gyr_cross_dn_dt[0];
        ayval = acceleration[1] - gyr_cross_gyr_cross_r_e[1] - wdot_cross_r_e[1] - R_wdot_cross_n[1] - R_gyr_cross_dn_dt[1];
        azval = acceleration[2] - gyr_cross_gyr_cross_r_e[2] - wdot_cross_r_e[2] - R_wdot_cross_n[2] - R_gyr_cross_dn_dt[2];

        if (debugMode)
        {
            ROS_DEBUG_THROTTLE(1.0,
                               "Centripetal (Eq. 12): gyr=(%.3f,%.3f,%.3f) wdot=(%.3f,%.3f,%.3f) n=(%.3f,%.3f,%.3f) dn/dt=(%.3f,%.3f,%.3f) "
                               "R*wdot×n=(%.3f,%.3f,%.3f) R*gyr×dn/dt=(%.3f,%.3f,%.3f)",
                               gyr[0], gyr[1], gyr[2], wdot[0], wdot[1], wdot[2], n_imu[0], n_imu[1], n_imu[2],
                               dn_dt[0], dn_dt[1], dn_dt[2], R_wdot_cross_n[0], R_wdot_cross_n[1], R_wdot_cross_n[2],
                               R_gyr_cross_dn_dt[0], R_gyr_cross_dn_dt[1], R_gyr_cross_dn_dt[2]);
        }
    }

    if (debugMode)
    {
        if (serialNr == SERIAL_0)
        {
            a0_comp[0] = axval;
            a0_comp[1] = ayval;
            a0_comp[2] = azval;
        }
        else if (serialNr == SERIAL_1)
        {
            a1_comp[0] = axval;
            a1_comp[1] = ayval;
            a1_comp[2] = azval;
        }
        else if (serialNr == SERIAL_2)
        {
            a2_comp[0] = axval;
            a2_comp[1] = ayval;
            a2_comp[2] = azval;
        }
    }

    // Gyros: assign latest value per IMU (like original)
    // Accels: accumulate across calls
    *gx_out = cur_gx;
    *gy_out = cur_gy;
    *gz_out = cur_gz;
    *ax_out += axval;
    *ay_out += ayval;
    *az_out += azval;
}

// -----------------------------------------------------------------------
// mergeGyros  —  weighted average helper
// -----------------------------------------------------------------------
static inline float mergeGyros(
    float g0, float g1, float g2, float g_prev = 0.0f)
{
    float res = 0.0f;
    if (use_serial0)
        res += g0;
    if (use_serial1)
        res += g1;
    if (use_serial2)
        res += g2;
    if (slow)
    {
        res += g_prev;
        res /= static_cast<float>(n_imus + 1);
    }
    else
    {
        res /= static_cast<float>(n_imus);
    }
    return res;
}

// -----------------------------------------------------------------------
// combineRAWData  —  average per-IMU compensated measurements
// Returns number of accumulated samples, or 0 if no data to process
// -----------------------------------------------------------------------
int combineRAWData(bool use0, bool use1, bool use2)
{
    int n = 0;
    mtx_n0.lock();
    mtx_n1.lock();
    mtx_n2.lock();
    if (use0)
        n += n0;
    if (use1)
        n += n1;
    if (use2)
        n += n2;

    /* --- Accelerometer average --- */
    if (n != 0)
    {
        ax = 0;
        ay = 0;
        az = 0;
        ax_alt = 0;
        ay_alt = 0;
        az_alt = 0;
        if (use0)
        {
            ax += ax0;
            ay += ay0;
            az += az0;
            ax_alt += a0_alt[0];
            ay_alt += a0_alt[1];
            az_alt += a0_alt[2];
        }
        if (use1)
        {
            ax += ax1;
            ay += ay1;
            az += az1;
            ax_alt += a1_alt[0];
            ay_alt += a1_alt[1];
            az_alt += a1_alt[2];
        }
        if (use2)
        {
            ax += ax2;
            ay += ay2;
            az += az2;
            ax_alt += a2_alt[0];
            ay_alt += a2_alt[1];
            az_alt += a2_alt[2];
        }
        ax /= n;
        ay /= n;
        az /= n;
        ax_alt /= n;
        ay_alt /= n;
        az_alt /= n;
        // reset accumulators
        a0_alt[0] = 0;
        a0_alt[1] = 0;
        a0_alt[2] = 0;
        a1_alt[0] = 0;
        a1_alt[1] = 0;
        a1_alt[2] = 0;
        a2_alt[0] = 0;
        a2_alt[1] = 0;
        a2_alt[2] = 0;
        n0 = 0;
        n1 = 0;
        n2 = 0;
        ax0 = 0;
        ay0 = 0;
        az0 = 0;
        ax1 = 0;
        ay1 = 0;
        az1 = 0;
        ax2 = 0;
        ay2 = 0;
        az2 = 0;
    }
    mtx_n0.unlock();
    mtx_n1.unlock();
    mtx_n2.unlock();

    /* --- Gyroscope average --- */
    if (firstRead < 4)
    {
        ROS_INFO("Initialising yaw axis...");
        ROS_INFO("combineRAWData: startup diagnostics: n=%d, ax0=%f, ay0=%f, az0=%f, ax1=%f, ay1=%f, az1=%f, ax2=%f, ay2=%f, az2=%f",
                 n, ax0, ay0, az0, ax1, ay1, az1, ax2, ay2, az2);
        ROS_INFO("combineRAWData: computed ax/ay/az before init = (%f, %f, %f)", ax, ay, az);
        gx = mergeGyros(gx0, gx1, gx2);
        gy = mergeGyros(gy0, gy1, gy2);
        gz = mergeGyros(gz0, gz1, gz2);
        ovrwrtOrientWithAcc(ax, ay, az, 0);
        changeAlt();
        ovrwrtOrientWithAcc(ax, ay, az, 0);
        changeAlt();
        firstRead = 4;
    }
    else if (!slow)
    {
        gx = mergeGyros(gx0, gx1, gx2);
        gy = mergeGyros(gy0, gy1, gy2);
        gz = mergeGyros(gz0, gz1, gz2);
    }
    else
    {
        gx = mergeGyros(gx0, gx1, gx2, gx);
        gy = mergeGyros(gy0, gy1, gy2, gy);
        gz = mergeGyros(gz0, gz1, gz2, gz);
    }

    return n;
}

// -----------------------------------------------------------------------
// calc_position
// -----------------------------------------------------------------------
void calc_position(float gx, float gy, float gz)
{
    float factorX = (0.05f * gx) * (0.05f * gx);
    float factorY = (0.05f * gy) * (0.05f * gy);
    float factorZ = (0.05f * gz) * (0.05f * gz);
    if (factorX > 1)
        factorX = 1;
    if (factorY > 1)
        factorY = 1;
    if (factorZ > 1)
        factorZ = 1;

    // Rotation matrix rows from current quaternion
    float r00 = 1 - 2 * (q2 * q2 + q3 * q3);
    float r01 = 2 * (q1 * q2 - q0 * q3);
    float r02 = 2 * (q1 * q3 + q0 * q2);
    float r10 = 2 * (q1 * q2 + q0 * q3);
    float r11 = 1 - 2 * (q1 * q1 + q3 * q3);
    float r12 = 2 * (q2 * q3 - q0 * q1);
    float r20 = 2 * (q1 * q3 - q0 * q2);
    float r21 = 2 * (q2 * q3 + q0 * q1);
    float r22 = 1 - 2 * (q1 * q1 + q2 * q2);

    // Gravity direction in robot frame (third column of R^T = third row of R)
    float grav_x = r20, grav_y = r21, grav_z = r22;

    if (fabs(ax - grav_x) > 0.02f)
        vel_x += (ax - grav_x) * 9.81f * dt;
    if (fabs(ay - grav_y) > 0.02f)
        vel_y += (ay - grav_y) * 9.81f * dt;
    if (fabs(az - grav_z) > 0.02f)
        vel_z += (az - grav_z) * 9.81f * dt;

    // Couple velocity to rotation speed
    vel_x *= std::max(factorY, factorZ);
    vel_y *= std::max(factorZ, factorX);
    vel_z *= std::max(factorX, factorY);

    // Velocity in world frame
    float vel_x_world = r00 * vel_x + r01 * vel_y + r02 * vel_z;
    float vel_y_world = r10 * vel_x + r11 * vel_y + r12 * vel_z;
    float vel_z_world = r20 * vel_x + r21 * vel_y + r22 * vel_z;
    if (setZ0 && !use_ground_normal)
        vel_z_world = 0;

    // Rolling-contact velocity from filtered rotation speed (dynamic model)
    float rot_x_world = r00 * gx_filtered + r01 * gy_filtered + r02 * gz_filtered;
    float rot_y_world = r10 * gx_filtered + r11 * gy_filtered + r12 * gz_filtered;
    float rot_z_world = r20 * gx_filtered + r21 * gy_filtered + r22 * gz_filtered;

    // Get contact normal in world frame (default to flat floor assumption)
    float normal_world_x = 0.0f, normal_world_y = 0.0f, normal_world_z = -1.0f;
    if (use_ground_normal && ground_normal_available)
    {
        mtx_gn.lock();
        normal_world_x = gn_x;
        normal_world_y = gn_y;
        normal_world_z = gn_z;
        mtx_gn.unlock();
    }

    // Dynamic model without flat floor assumption: v = r * (ω × n)
    float vel_x_world_rot = r * (rot_y_world * normal_world_z - rot_z_world * normal_world_y);
    float vel_y_world_rot = r * (rot_z_world * normal_world_x - rot_x_world * normal_world_z);
    float vel_z_world_rot = r * (rot_x_world * normal_world_y - rot_y_world * normal_world_x);

    // Evaluation path (unfiltered gyro)
    float rot_x_world_alt = r00 * gx + r01 * gy + r02 * gz;
    float rot_y_world_alt = r10 * gx + r11 * gy + r12 * gz;
    float rot_z_world_alt = r20 * gx + r21 * gy + r22 * gz;

    float vel_x_world_rot_alt = r * (rot_y_world_alt * normal_world_z - rot_z_world_alt * normal_world_y);
    float vel_y_world_rot_alt = r * (rot_z_world_alt * normal_world_x - rot_x_world_alt * normal_world_z);
    float vel_z_world_rot_alt = r * (rot_x_world_alt * normal_world_y - rot_y_world_alt * normal_world_x);

    // Debugging: log key values when using dynamic ground normal
    if (use_ground_normal && ground_normal_available)
    {
        // ROS_INFO_THROTTLE(2.0,
        //                   "DBG calc_position: r=%.3f q=(%.3f,%.3f,%.3f,%.3f) gn=(%.3f,%.3f,%.3f) normal_world=(%.3f,%.3f,%.3f) rot_f=(%.3f,%.3f,%.3f) vel_rot=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)",
        //                   r, q0, q1, q2, q3, gn_x, gn_y, gn_z, normal_world_x, normal_world_y, normal_world_z,
        //                   rot_x_world, rot_y_world, rot_z_world,
        //                   vel_x_world_rot, vel_y_world_rot, vel_z_world_rot,
        //                   vel_x_world, vel_y_world, vel_z_world);

        if (!std::isfinite(normal_world_x) || !std::isfinite(normal_world_y) || !std::isfinite(normal_world_z) ||
            !std::isfinite(rot_x_world) || !std::isfinite(rot_y_world) || !std::isfinite(rot_z_world))
        {
            ROS_WARN_THROTTLE(2.0, "DBG calc_position: non-finite values detected in rotation/normal computation");
        }
    }

    pXAlt += vel_x_world_rot_alt * dt;
    pYAlt += vel_y_world_rot_alt * dt;
    pZAlt += vel_z_world_rot_alt * dt;

    // Limit velocity magnitude to rotation-based velocity magnitude (prevent error accumulation)
    float mean_vel_world = sqrtf(vel_x_world_rot * vel_x_world_rot + vel_y_world_rot * vel_y_world_rot + vel_z_world_rot * vel_z_world_rot);
    if (fabs(vel_z_world) > mean_vel_world)
        vel_z_world = std::copysign(1.0f, vel_z_world) * mean_vel_world;
    // Cap accel-integrated velocity to 110% of rotation-based velocity
    float trust = 0.1f;

    // if (fabs(vel_x_world) > fabs(vel_x_world_rot))
    //     vel_x_world = std::min((1 + trust) * vel_x_world_rot, vel_x_world);
    // if (fabs(vel_y_world) > fabs(vel_y_world_rot))
    //     vel_y_world = std::min((1 + trust) * vel_y_world_rot, vel_y_world);
    // if (fabs(vel_z_world) > fabs(vel_z_world_rot))
    //     vel_z_world = std::min((1 + trust) * vel_z_world_rot, vel_z_world);

    // adjusted velocity capping which accounts for negative velocities
    float max_vel_x = (1.0f + trust) * fabs(vel_x_world_rot);
    vel_x_world = std::max(-max_vel_x, std::min(max_vel_x, vel_x_world)); // clamp vel_x_world to [-max_vel_x, max_vel_x]; returns vel_x_world if it is within [-max_vel_x, max_vel_x]

    float max_vel_y = (1.0f + trust) * fabs(vel_y_world_rot);
    vel_y_world = std::max(-max_vel_y, std::min(max_vel_y, vel_y_world));

    float max_vel_z = (1.0f + trust) * fabs(vel_z_world_rot);
    vel_z_world = std::max(-max_vel_z, std::min(max_vel_z, vel_z_world));

    // Subtract contact-normal component from rolling velocity
    // float vn2 = vel_z_world * vel_z_world; // component along normal
    // vel_x_world_rot = std::copysign(1.0f, vel_x_world_rot) * sqrtf(std::max(0.0f, vel_x_world_rot * vel_x_world_rot - vn2));
    // vel_y_world_rot = std::copysign(1.0f, vel_y_world_rot) * sqrtf(std::max(0.0f, vel_y_world_rot * vel_y_world_rot - vn2));
    // vel_z_world_rot = std::copysign(1.0f, vel_z_world_rot) * sqrtf(std::max(0.0f, vel_z_world_rot * vel_z_world_rot - vn2));

    vel_x_world_rot = std::copysign(1.0f, vel_x_world_rot) * sqrtf(std::max(0.0f, vel_x_world_rot * vel_x_world_rot));
    vel_y_world_rot = std::copysign(1.0f, vel_y_world_rot) * sqrtf(std::max(0.0f, vel_y_world_rot * vel_y_world_rot));
    vel_z_world_rot = std::copysign(1.0f, vel_z_world_rot) * sqrtf(std::max(0.0f, vel_z_world_rot * vel_z_world_rot));

    if (vel_x_world_rot * vel_x_world_rot > 0.001f || vel_y_world_rot * vel_y_world_rot > 0.001f || vel_z_world_rot * vel_z_world_rot > 0.001f)
    {
        px += vel_x_world_rot * dt;
        py += vel_y_world_rot * dt;
        pz += vel_z_world_rot * dt;
    }
}

// -----------------------------------------------------------------------
// apply_attitude_filter
// -----------------------------------------------------------------------
void apply_attitude_filter(double stamp_ms, bool reuse_dt)
{
    if (!reuse_dt)
    {
        dt = static_cast<float>((stamp_ms - lastTime) / 1000.0);
        lastTime = stamp_ms;
    }
    // Validate dt
    if (!std::isfinite(dt) || dt <= 0.0f)
    {
        ROS_WARN("apply_attitude_filter: invalid dt=%f, skipping filter step", dt);
        return;
    }

    lastq0 = q0;
    lastq1 = q1;
    lastq2 = q2;
    lastq3 = q3;

    Vec3 acc(ax, ay, az);
    Vec3 gyro(gx, gy, gz);
    quaternion res = estimator_instance->filter(gyro, acc, dt);
    // Defensive check: ensure estimator did not return NaNs or a near-zero quaternion
    if (!std::isfinite(res.w) || !std::isfinite(res.x) || !std::isfinite(res.y) || !std::isfinite(res.z))
    {
        ROS_WARN("apply_attitude_filter: estimator returned non-finite quaternion, restoring previous state");
        q0 = lastq0;
        q1 = lastq1;
        q2 = lastq2;
        q3 = lastq3;
        estimator_instance->override_state(q0, q1, q2, q3);
        return;
    }
    double qnorm = fabs(res.w) + fabs(res.x) + fabs(res.y) + fabs(res.z);
    if (qnorm < 1e-6)
    {
        ROS_WARN("apply_attitude_filter: estimator returned near-zero quaternion, restoring previous state");
        q0 = lastq0;
        q1 = lastq1;
        q2 = lastq2;
        q3 = lastq3;
        estimator_instance->override_state(q0, q1, q2, q3);
        return;
    }

    q0 = res.w;
    q1 = res.x;
    q2 = res.y;
    q3 = res.z;

    // Compute filtered rotation speed via axis-angle from relative quaternion
    const quaternion qLast(lastq0, lastq1, lastq2, lastq3);
    const quaternion qFiltered(q0, q1, q2, q3);
    quaternion qLastInv;
    quaternion_inverse(&qLast, &qLastInv);
    quaternion relative_rotation;
    quaternion_multiply(&qFiltered, &qLastInv, &relative_rotation);
    double axis[3];
    double angle;
    quaternion_to_axis_angle(&relative_rotation, axis, &angle);
    angle *= precalc_180_BY_M_PI;
    gx_filtered = axis[0] * angle;
    gy_filtered = axis[1] * angle;
    gz_filtered = axis[2] * angle;

    // Compute position
    if (!reuse_dt)
    {
        calc_position(gx, gy, gz);
    }
}

// -----------------------------------------------------------------------
// initialise quaternion from accelerometer
// -----------------------------------------------------------------------
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw)
{
    float acc_mag_sq = ax * ax + ay * ay + az * az;
    // Guard against near-zero acceleration
    if (acc_mag_sq < 1e-6f)
    {
        ROS_WARN("ovrwrtOrientWithAcc: acceleration near zero (mag^2=%e), skipping initialization", acc_mag_sq);
        return;
    }

    float recipNorm = 1.0f / sqrtf(acc_mag_sq);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    float acc_roll = atan2f(ay, az);

    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    quatFromEuler(&q0, &q1, &q2, &q3, acc_roll, acc_pitch, yaw);
    ROS_INFO("Initial: Roll: %f Pitch: %f Yaw: %f", acc_roll * precalc_180_BY_M_PI, acc_pitch * precalc_180_BY_M_PI, yaw * precalc_180_BY_M_PI);
}

// -----------------------------------------------------------------------
// receive updated ground normal from global ground finder node
// -----------------------------------------------------------------------
static void groundNormalCallback(const ground_finder_msgs::ScoredNormalStamped::ConstPtr &msg)
{
    if (use_ground_normal)
    {
        geometry_msgs::Vector3Stamped n;
        n.header = msg->header;
        n.vector = msg->normal;

        float nx = n.vector.x;
        float ny = n.vector.y;
        float nz = n.vector.z;
        const float n_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (n_norm > 1e-6f)
        {
            nx /= n_norm;
            ny /= n_norm;
            nz /= n_norm;
        }

        mtx_gn.lock();
        gn_x = nx;
        gn_y = ny;
        gn_z = nz;
        ground_normal_available = true;
        // ROS_INFO_THROTTLE(2.0, "Ground normal in map_lio: (%.3f, %.3f, %.3f)", gn_x, gn_y, gn_z);
        mtx_gn.unlock();
    }
}

// -----------------------------------------------------------------------
// receive updated ground normal (in IMU frame) for centripetal compensation
// -----------------------------------------------------------------------
static void groundNormalCentripetalCallback(const ground_finder_msgs::ScoredNormalStamped::ConstPtr &msg)
{
    if (use_normal_centripetal)
    {
        geometry_msgs::Vector3Stamped n_src, n_imu;
        n_src.header = msg->header;
        n_src.vector = msg->normal;

        // Lazy TF lookup: attempt on first call if not yet loaded
        static bool tf_lookup_attempted = false;
        if (!ground_normal_tf_ready && !tf_lookup_attempted)
        {
            tf_lookup_attempted = true;
            try
            {
                if (tf_buffer_.canTransform(ground_normal_target_frame, ground_normal_source_frame, ros::Time(0), ros::Duration(0.5)))
                {
                    ground_normal_tf = tf_buffer_.lookupTransform(ground_normal_target_frame, ground_normal_source_frame,
                                                                  ros::Time(0), ros::Duration(0.1));
                    ground_normal_tf_ready = true;
                    ROS_INFO("Loaded static TF for ground normal rotation (lazy): %s -> %s",
                             ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str());
                }
                else
                {
                    ROS_WARN("TF not available for ground normal rotation (lazy attempt) (%s -> %s). Using incoming normal as-is.",
                             ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str());
                }
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_WARN("Failed to lookup TF for ground normal rotation (lazy attempt) (%s -> %s): %s. Using incoming normal as-is.",
                         ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str(), ex.what());
            }
        }

        // TF transformation from pandar_frame (source) to imu_frame (target)
        if (ground_normal_tf_ready)
        {
            try
            {
                tf2::doTransform(n_src, n_imu, ground_normal_tf);
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(2.0, "Failed to transform ground normal (centripetal): %s", ex.what());
                n_imu = n_src;
            }
        }
        else
        {
            n_imu = n_src;
        }

        float nx = n_imu.vector.x;
        float ny = n_imu.vector.y;
        float nz = n_imu.vector.z;
        const float n_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (n_norm > 1e-6f)
        {
            nx /= n_norm;
            ny /= n_norm;
            nz /= n_norm;
        }

        mtx_gn_imu.lock();
        gn_x_imu_prev = gn_x_imu;
        gn_y_imu_prev = gn_y_imu;
        gn_z_imu_prev = gn_z_imu;
        gn_x_imu = nx;
        gn_y_imu = ny;
        gn_z_imu = nz;
        ground_normal_centripetal_available = true;
        // ROS_INFO_THROTTLE(0.1, "Ground normal (centripetal) in imu_frame: (%.3f, %.3f, %.3f)", gn_x_imu, gn_y_imu, gn_z_imu);
        mtx_gn_imu.unlock();
    }
}

// -----------------------------------------------------------------------
// ROS subscriber callbacks  (one per IMU)
//
// Each callback applies centripetal compensation and accumulates into
// the per-IMU running totals. main loop then calls combineRAWData()
// to produce the averaged gx/gy/gz and ax/ay/az.
// -----------------------------------------------------------------------
static void imu0Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // static int call_count = 0;
    //  if (++call_count % 100 == 1) // Log every 100 calls
    //      ROS_INFO_THROTTLE(5.0, "imu0Callback: received message #%d, acc=(%.2f, %.2f, %.2f)",
    //                        call_count, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    double acc[3] = {msg->linear_acceleration.x,
                     msg->linear_acceleration.y,
                     msg->linear_acceleration.z};
    double gyr[3] = {msg->angular_velocity.x,
                     msg->angular_velocity.y,
                     msg->angular_velocity.z};
    mtx_n0.lock();
    last_stamp0_s = msg->header.stamp.toSec();
    imu0_ready = true;
    n0++;
    setValsAndCompensateCentripetal(SERIAL_0, acc, gyr, smooth_deriv_kernel0,
                                    &gx0, &gy0, &gz0, &ax0, &ay0, &az0);
    mtx_n0.unlock();
}

static void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // static int call_count = 0;
    // if (++call_count % 100 == 1)
    //     ROS_INFO_THROTTLE(5.0, "imu1Callback: received message #%d, acc=(%.2f, %.2f, %.2f)",
    //                       call_count, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    double acc[3] = {msg->linear_acceleration.x,
                     msg->linear_acceleration.y,
                     msg->linear_acceleration.z};
    double gyr[3] = {msg->angular_velocity.x,
                     msg->angular_velocity.y,
                     msg->angular_velocity.z};
    mtx_n1.lock();
    last_stamp1_s = msg->header.stamp.toSec();
    imu1_ready = true;
    n1++;
    setValsAndCompensateCentripetal(SERIAL_1, acc, gyr, smooth_deriv_kernel1,
                                    &gx1, &gy1, &gz1, &ax1, &ay1, &az1);
    mtx_n1.unlock();
}

static void imu2Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // static int call_count = 0;
    // if (++call_count % 100 == 1)
    //     ROS_INFO_THROTTLE(5.0, "imu2Callback: received message #%d, acc=(%.2f, %.2f, %.2f)",
    //                       call_count, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    double acc[3] = {msg->linear_acceleration.x,
                     msg->linear_acceleration.y,
                     msg->linear_acceleration.z};
    double gyr[3] = {msg->angular_velocity.x,
                     msg->angular_velocity.y,
                     msg->angular_velocity.z};
    mtx_n2.lock();
    last_stamp2_s = msg->header.stamp.toSec();
    imu2_ready = true;
    n2++;
    setValsAndCompensateCentripetal(SERIAL_2, acc, gyr, smooth_deriv_kernel2,
                                    &gx2, &gy2, &gz2, &ax2, &ay2, &az2);
    mtx_n2.unlock();
}

// -----------------------------------------------------------------------
// parse ROS parameters for the filter node
// -----------------------------------------------------------------------
int argumentHandler(ros::NodeHandle &nh)
{
    nh.param<float>("sphere_radius", r, 0.145f);
    if (r <= 0.0f)
        ROS_WARN("sphere_radius not set (%f)!", r);

    // Filter gains / covariances
#if defined(MAHONY)
    nh.param<double>("mahony_kp", Kp, 1.0);
    nh.param<double>("mahony_ki", Ki, 0.001);
#elif defined(QEKF) || defined(UKF)
    nh.param<float>("sigma_g", sigma_gyr, 0.001f);
    nh.param<float>("sigma_a", sigma_a, 0.001f);
    nh.param<float>("P_init", P_init, 0.01f);
#else
    nh.param<float>("jasper_gain", gain_, 0.0f);
    nh.param<float>("jasper_alpha", alpha, 0.02f);
    nh.param<float>("jasper_autogain", autogain, 0.0f);
    if (autogain > 0)
    {
        alpha = autogain;
        ROS_INFO("autogain = %f -> gain = 0, alpha init = %f", autogain, autogain * 0.1f);
    }
    gain_min = gain_;
#endif
#if defined(UKF)
    nh.param<float>("ukf_alpha", alpha, 0.001f);
    nh.param<float>("ukf_beta", beta, 2.0f);
    nh.param<float>("ukf_kappa", kappa, 0.0f);
#endif

    nh.param<bool>("jasper_slow", slow, false);
    nh.param<bool>("jasper_forceflat", setZ0, true);
    nh.param<bool>("jasper_debug", debugMode, false);
    nh.param<std::string>("topicName", topicName, std::string("posePub_merged"));

    // Rotation-speed threshold for position integration
    nh.param<float>("small_omega", small_omega2, 0.1f);
    small_omega2 *= small_omega2;

    // Active IMUs and their serial numbers (needed by setValsAndCompensateCentripetal
    // to select the correct lever-arm vector)
    nh.param<int>("jasper_serial0", SERIAL_0, 0);
    nh.param<int>("jasper_serial1", SERIAL_1, 0);
    nh.param<int>("jasper_serial2", SERIAL_2, 0);
    nh.param<bool>("use_serial0", use_serial0, false);
    nh.param<bool>("use_serial1", use_serial1, false);
    nh.param<bool>("use_serial2", use_serial2, false);
    if (use_serial0 && SERIAL_0 == 0)
        use_serial0 = false;
    if (use_serial1 && SERIAL_1 == 0)
        use_serial1 = false;
    if (use_serial2 && SERIAL_2 == 0)
        use_serial2 = false;
    n_imus = (use_serial0 ? 1 : 0) + (use_serial1 ? 1 : 0) + (use_serial2 ? 1 : 0);

    /*
        EXTRINSIC PARAMETERS

    The extrinsic parameters are defined using the coordinate system of the IMU.
    That is, in the config file, you must specify the vector to the center point of the ball, as seen from
    each individual IMU.
    */
    float tmpx, tmpy, tmpz;
    nh.param<float>("imu0_x", tmpx, 0.0f);
    nh.param<float>("imu0_y", tmpy, 0.0f);
    nh.param<float>("imu0_z", tmpz, 0.0f);
    pos_serial0 = new float[3]{tmpx, tmpy, tmpz};
    nh.param<float>("imu1_x", tmpx, 0.0f);
    nh.param<float>("imu1_y", tmpy, 0.0f);
    nh.param<float>("imu1_z", tmpz, 0.0f);
    pos_serial1 = new float[3]{tmpx, tmpy, tmpz};
    nh.param<float>("imu2_x", tmpx, 0.0f);
    nh.param<float>("imu2_y", tmpy, 0.0f);
    nh.param<float>("imu2_z", tmpz, 0.0f);
    pos_serial2 = new float[3]{tmpx, tmpy, tmpz};

    // -----------------------------------------------------------------------
    // Smoothed-derivative kernel (for the tangential term in centripetal comp.)
    // -----------------------------------------------------------------------
    int imu_data_rate = IMU_DATA_RATE_DEFAULT;
    float freq_cut;
    nh.param<int>("jasper_imu_rate", imu_data_rate, IMU_DATA_RATE_DEFAULT);
    nh.param<float>("jasper_lowpass_freq", freq_cut, 10.0f);
    float sigma = 1.0f / (2.0f * M_PI * freq_cut);
    float imu_data_dt = 1.0f / imu_data_rate;
    float n_data_dt = 1.0f / 20; // 20Hz da Lidar daten in 20hz kommen (TODO: checken)
    int win_size = static_cast<int>(6 * sigma / imu_data_dt);
    win_size = (win_size % 2 == 0) ? win_size + 1 : win_size;
    ROS_WARN("Derivative kernel: window=%d, cutoff=%.1f Hz, sigma=%.4f, dt=%.4f",
             win_size, freq_cut, sigma, imu_data_dt);

    // One independent kernel per IMU so their ring-buffer histories never mix
    smooth_deriv_kernel0 = new SmoothedDerivative3D(win_size, sigma, imu_data_dt);
    smooth_deriv_kernel1 = new SmoothedDerivative3D(win_size, sigma, imu_data_dt);
    smooth_deriv_kernel2 = new SmoothedDerivative3D(win_size, sigma, imu_data_dt);
    smooth_deriv_kernelN = new SmoothedDerivative3D(win_size, sigma, n_data_dt);

    // -----------------------------------------------------------------------
    // Ground normal parameters
    // -----------------------------------------------------------------------
    nh.param<bool>("use_ground_normal", use_ground_normal, true);
    nh.param<bool>("use_normal_centripetal", use_normal_centripetal, false);

    if (use_ground_normal)
    {
        ground_normal_topic = "/global_ground_finder/smoothed_scored_normal";
        ground_normal_sub = nh.subscribe(ground_normal_topic, 1, groundNormalCallback);
        ROS_INFO("Using ground normal for pose estimation. Subscribing to topic %s", ground_normal_topic.c_str());
    }

    if (use_normal_centripetal)
    {
        // Initialize tf2 listener for frame transformations (needed for centripetal compensation)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        try
        {
            if (tf_buffer_.canTransform(ground_normal_target_frame, ground_normal_source_frame, ros::Time(0), ros::Duration(2.0)))
            {
                ground_normal_tf = tf_buffer_.lookupTransform(ground_normal_target_frame, ground_normal_source_frame,
                                                              ros::Time(0), ros::Duration(0.1));
                ground_normal_tf_ready = true;
                ROS_INFO("Loaded static TF for ground normal rotation: %s -> %s",
                         ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str());
            }
            else
            {
                ROS_WARN("TF not available for ground normal rotation (%s -> %s). Using incoming normal as-is.",
                         ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str());
            }
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN("Failed to lookup TF for ground normal rotation (%s -> %s): %s. Using incoming normal as-is.",
                     ground_normal_source_frame.c_str(), ground_normal_target_frame.c_str(), ex.what());
        }

        std::string ground_normal_centripetal_topic = "/global_ground_finder/scored_normal_pandar";
        ground_normal_centripetal_sub = nh.subscribe(ground_normal_centripetal_topic, 1, groundNormalCentripetalCallback);
        ROS_INFO("Using ground normal for centripetal compensation. Subscribing to topic %s", ground_normal_centripetal_topic.c_str());
    }

    return 0;
}

// -----------------------------------------------------------------------
// SIGINT handler
// -----------------------------------------------------------------------
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

// -----------------------------------------------------------------------
// main
// -----------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_filter", ros::init_options::NoSigintHandler);
    ROS_INFO("imu_filter node starting...");
    signal(SIGINT, mySigIntHandler);

    ros::NodeHandle nH;
    argumentHandler(nH);

    // ---- Instantiate compile-time-selected attitude estimator ----
#if defined(MAHONY)
    estimator_instance = new MahonyFilter(Kp, Ki);
#elif defined(QEKF)
    estimator_instance = new ExtendedKalmanFilter(sigma_gyr, sigma_a, P_init);
#elif defined(UKF)
    estimator_instance = new UnscentedKalmanFilter();
#else
    estimator_instance = new AutogainFilter(gain_min, alpha, autogain);
#endif

    // ---- Subscribe to calibrated IMU topics ----
    ros::Subscriber sub0, sub1, sub2;
    if (use_serial0)
        sub0 = nH.subscribe("/imu/0/calib", 1000, imu0Callback);
    if (use_serial1)
        sub1 = nH.subscribe("/imu/1/calib", 1000, imu1Callback);
    if (use_serial2)
        sub2 = nH.subscribe("/imu/2/calib", 1000, imu2Callback);

    // ---- Advertise output topics ----
    ros::Publisher estimator_pub = nH.advertise<state_estimator_msgs::Estimator>(topicName, 1000);
    ros::Publisher estimator_raw_pub = nH.advertise<state_estimator_msgs::Estimator>(topicName + "_raw", 1000);

    // Initialise output message
    seq = 0;
    output_msg.header.frame_id = "map";
    output_msg.header.seq = seq++;
    output_msg.header.stamp = ros::Time::now();
    output_msg.pose.pose.position.x = 0;
    output_msg.pose.pose.position.y = 0;
    output_msg.pose.pose.position.z = 0;
    output_msg.pose.pose.orientation.w = q0;
    output_msg.pose.pose.orientation.x = q1;
    output_msg.pose.pose.orientation.y = q2;
    output_msg.pose.pose.orientation.z = q3;

    // Read publish rate (same param the hardware node uses so the two nodes
    // can be configured from the same launch file)
    int data_rate = DATA_RATE_DEFAULT;
    {
        int temp;
        nH.param<int>("jasper_pub_rate", temp, DATA_RATE_DEFAULT);
        if (temp > 0 && temp < 501)
            data_rate = temp;
    }
    ros::Rate loop_rate(data_rate);

    // Wait until every active IMU has delivered at least one message via callbacks.
    auto allReady = [&]()
    {
        return (!use_serial0 || imu0_ready) && (!use_serial1 || imu1_ready) && (!use_serial2 || imu2_ready);
    };
    while (!allReady() && !g_request_shutdown)
    {
        ROS_WARN_THROTTLE(1.0, "Waiting for calibrated IMU topics");
        ros::spinOnce();
    }
    // Seed lastTime from the first real message stamps so the first dt is meaningful
    {
        double seed = 0.0;
        int n = 0;
        if (use_serial0 && last_stamp0_s > 0.0)
        {
            seed += last_stamp0_s;
            n++;
        }
        if (use_serial1 && last_stamp1_s > 0.0)
        {
            seed += last_stamp1_s;
            n++;
        }
        if (use_serial2 && last_stamp2_s > 0.0)
        {
            seed += last_stamp2_s;
            n++;
        }
        lastTime = (n > 0) ? (seed / n * 1000.0) : (ros::Time::now().toSec() * 1000.0);
    }
    ROS_INFO("IMU data received - filter starting.");

    /*--------------------------------------------------------------------
                            Main processing loop
    --------------------------------------------------------------------*/
    while (ros::ok() && !g_request_shutdown)
    {
        ros::spinOnce();

        int n_combined = combineRAWData(use_serial0, use_serial1, use_serial2);

        // Skip filter if no data accumulated yet (startup race condition)
        if (n_combined == 0)
        {
            loop_rate.sleep();
            continue;
        }

        // Compute combined timestamp as the average of the latest header
        // stamps from each active IMU.  Using message stamps (not
        // ros::Time::now()) makes bag replay reproduce the correct dt.
        double combined_stamp_ms = 0.0;
        int n_active = 0;
        if (use_serial0 && last_stamp0_s > 0.0)
        {
            combined_stamp_ms += last_stamp0_s;
            n_active++;
        }
        if (use_serial1 && last_stamp1_s > 0.0)
        {
            combined_stamp_ms += last_stamp1_s;
            n_active++;
        }
        if (use_serial2 && last_stamp2_s > 0.0)
        {
            combined_stamp_ms += last_stamp2_s;
            n_active++;
        }
        if (n_active > 0)
            combined_stamp_ms = combined_stamp_ms / n_active * 1000.0;
        else
            combined_stamp_ms = ros::Time::now().toSec() * 1000.0; // fallback

        apply_attitude_filter(combined_stamp_ms, /*reuse_dt=*/false);

        // ---- Publish main (compensated) estimate ----
        ms_to_ros_stamp(combined_stamp_ms, output_msg.header.stamp);
        output_msg.header.frame_id = "map_imu";
        output_msg.header.seq = seq++;
        output_msg.pose.header = output_msg.header;
        output_msg.pose.pose.position.x = px;
        output_msg.pose.pose.position.y = py;
        output_msg.pose.pose.position.z = pz;
        output_msg.pose.pose.orientation.w = q0;
        output_msg.pose.pose.orientation.x = q1;
        output_msg.pose.pose.orientation.y = q2;
        output_msg.pose.pose.orientation.z = q3;
        output_msg.imu.header = output_msg.header;
        output_msg.imu.orientation = output_msg.pose.pose.orientation;
        output_msg.imu.angular_velocity.x = gx;
        output_msg.imu.angular_velocity.y = gy;
        output_msg.imu.angular_velocity.z = gz;
        output_msg.imu.linear_acceleration.x = ax;
        output_msg.imu.linear_acceleration.y = ay;
        output_msg.imu.linear_acceleration.z = az;
        estimator_pub.publish(output_msg);

        // ---- Publish evaluation (raw / uncompensated) estimate ----
        changeAlt();
        ax = ax_alt;
        ay = ay_alt;
        az = az_alt;
        apply_attitude_filter(combined_stamp_ms, /*reuse_dt=*/true);
        output_msg.header.frame_id = "map_raw";
        output_msg.pose.pose.position.x = px;
        output_msg.pose.pose.position.y = py;
        output_msg.pose.pose.position.z = pz;
        output_msg.pose.pose.orientation.w = q0;
        output_msg.pose.pose.orientation.x = q1;
        output_msg.pose.pose.orientation.y = q2;
        output_msg.pose.pose.orientation.z = q3;
        output_msg.imu.orientation = output_msg.pose.pose.orientation;
        output_msg.imu.angular_velocity.x = gx;
        output_msg.imu.angular_velocity.y = gy;
        output_msg.imu.angular_velocity.z = gz;
        output_msg.imu.linear_acceleration.x = ax;
        output_msg.imu.linear_acceleration.y = ay;
        output_msg.imu.linear_acceleration.z = az;
        estimator_raw_pub.publish(output_msg);
        changeAlt();

        loop_rate.sleep();
    }

    ROS_WARN("imu_filter shutdown - goodbye!");
    ros::shutdown();
    return 0;
}