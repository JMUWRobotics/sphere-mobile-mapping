// System
#include <memory>
#include <queue>
#include <cmath>

// Math
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Custom msg
#include <state_estimator_msgs/Estimator.h>

/** ---------------------------------------
 * ------------Rosparam server defines -----
 * ----------------------------------------*/

int imu_rate, cam_rate;   // in Hz
int fast_rate, slow_rate; // Redefinition, same as above
int n_acc;                // Accumulation window of slower pose stream
double r_sphere;          // Radius of sphere in m
double large_omega;       // in rad/s, rotation speed above is considered large
double small_omega;       // in rad/s, rotation speed below is considered small

/** ---------------------------------------
 * ------------Internal variables----------
 * ----------------------------------------*/

// Which pose stream to use for interpolation (the faster one)
enum INTERPOLATE_STREAM {
    CAM,    
    IMU,
    NONE
} interpolate;

// Switches the motion model
enum MOTION_MODEL {
    ROLLING_MODE,
    CAMERA_PASSTROUGH
} model;

std::shared_ptr<ros::Rate> node_frequency;

// Flags to initialize pose streams
bool initialized_cam, initialized_imu;

// Input state estimator objects for IMU, CAM
state_estimator_msgs::Estimator last_imu_pose, last_cam_pose;

// Output message of kalman filter: geometry message stamped pose 
geometry_msgs::PoseStamped filtered_pose_msg;

// tf Pose objects for calculating interpolated delta
tf::Pose pose_interpolated, last_pose_interpolated;

// Pose accumulator for the faster stream
std::queue<state_estimator_msgs::Estimator> accumulator;

// Callback queue for the faster stream. For more than 2 estimators this should be a vector of CallbackQueues.
ros::CallbackQueue callbacks_fast;

// Transformation between IMU and Camera pose frame (FROM Imu TO Cam)
tf::StampedTransform tf_map_imu2cam, tf_axes_imu2cam;

ros::Publisher filtered_pose_pub;

// Save last interpolated confidence level of camera
float last_translated_confidence;
float scalingFactorCamera;
float scalingFactorImu;

// Debug stuff
ros::Publisher debug_pose_pub;
geometry_msgs::PoseStamped debug_pose_msg;
bool publish_debug_topic;

uint32_t sequence = 0; // Sequence number for publish msg

// Global defines as objects on heap
const tf::Vector3 origin(0, 0, 0);
const tf::Quaternion rot_zero(0, 0, 0, 1);

// above this value should be considered fast rotation in (rad/s)^2
double large_omega2; // (rad/s)^2 
double small_omega2; // (rad/s)^2 
// maximum and minimum scaling factor for covariance matrices
const float MIN_SCALING_FACTOR = 0.1;
const float MAX_SCALING_FACTOR = 1000.0;
const double small_number = 0.00001;

/** ---------------------------------------
 * ------------Supporting methods----------
 * ----------------------------------------*/

// Advances the internal queue such that the timestamp argument is between front and end
inline void waitForAccumulator(double t)
{

    // Wait for the queue to be not empty
    while (accumulator.size() < 1 && ros::ok()) {
        callbacks_fast.callOne(ros::WallDuration());
    }

    // Check if lagging behind
    if (t < accumulator.front().header.stamp.toSec()) {
        ROS_INFO("Lagging behind, skipping frame for interpolation");
        return;
    }

    // Wait for the query time to be between the accumulator times
    while (!(accumulator.front().header.stamp.toSec() < t && t < accumulator.back().header.stamp.toSec()) && ros::ok()) {
        // If interpolation is disabled, all msgs have the same queue
        if (interpolate == NONE)
            ros::spinOnce();
        else
            callbacks_fast.callOne(ros::WallDuration());

        node_frequency->sleep();
    }
}

// Push a message to the internal queue, removes the oldest entry if full. 
inline void pushToAccumulator(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    accumulator.push(*m);
    if (accumulator.size() > n_acc)
        accumulator.pop();
}

// Helper to calculate the diff transform between two poses
inline tf::Transform pose_diff(const state_estimator_msgs::Estimator::ConstPtr &m, state_estimator_msgs::Estimator &last_pose_msg)
{
    tf::Stamped<tf::Pose> current_pose, last_pose;
    tf::poseStampedMsgToTF(m->pose, current_pose);
    tf::poseStampedMsgToTF(last_pose_msg.pose, last_pose);
    return current_pose.inverseTimes(last_pose);
}

// Converts geometry_msgs::Quaternion to roll, pitch, and yaw angles.
inline void quat2RPY(const geometry_msgs::Quaternion &q, double& r, double &p, double &y)
{
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    tf::Matrix3x3(q2).getRPY(r, p, y);
}

/* Confidence level (0 = Failed, 1 = Low, 2 = Medium, 3 = High confidence) will be translated to a factor, 
   which the pose variance will be multiplied with
   low confidence -> higher variance -> less influence in KF */
inline uint32_t confidenceTranslator(const uint8_t &confidence){
    return pow(10, 3 - confidence);
}

/*
Bigger scaling factor -> More uncertanty -> Less weight in KF
Lesser scaling factor -> Less uncertanty -> More weight in KF
*/
inline float getScalingFactor(const tf::Vector3 &angular_vel, bool isCam){

    float scalingFactor = 1.0;

    if(isCam){
        // Camera uncertanty should be HIGHER when rolling fast 
        scalingFactor = last_translated_confidence * exp(angular_vel.length2());
    } else {
        // IMU uncertanty should be LOWER when rolling fast (compared to camera)
        scalingFactor = (1000.0 / last_translated_confidence) / exp(angular_vel.length2());
    } 

    // Clamp to be between MIN and MAX value
    scalingFactor = std::min(std::max(scalingFactor, MIN_SCALING_FACTOR), MAX_SCALING_FACTOR);
    return scalingFactor;
};

// Manually overwrite the current orientation to reset accumulated error
inline void overwriteAttitude(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    double r_new, p_new, y_new, r_old, p_old, y_old;
    quat2RPY(filtered_pose_msg.pose.orientation, r_old, p_old, y_old);
    quat2RPY(m->pose.pose.orientation, r_new, p_new, y_new);
    geometry_msgs::Quaternion q_new;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(r_new, p_new, y_old), q_new);
    filtered_pose_msg.pose.orientation = q_new;
}

inline bool getRollingModeActive()
{   
    return model == ROLLING_MODE;
}

/*
Sets the rolling mode or passtrough mode in a fuzzy fashion.
Handles reset of orientation to prevent accumulation of error.
Parameters:
wx - global rotation speed around x
wy - global rotation speed around y
wz - global rotation speed around z
m - Pose msg to be used if attitude reset applies 
*/  
inline bool decideRollingMode(tf::Vector3 &global_ang_vel, const state_estimator_msgs::Estimator::ConstPtr &m) 
{   
    double wx = global_ang_vel.x(), wy = global_ang_vel.y(), wz = global_ang_vel.z();
    // We want to set rolling mode active IF rotations are large (ignore global rotation around gravity)
    if (wx*wx + wy*wy > large_omega2 && model != ROLLING_MODE) {
        // If we are not already in rolling mode
        model = ROLLING_MODE;
        ROS_INFO("Kalman filter motion model swapped to ROLLING");
    }
    // We do not want to set rolling mode, but passtrough instead IF rotations are small enough
    else if (model != CAMERA_PASSTROUGH 
     && wx*wx + wy*wy + wz*wz < small_omega2    
     && scalingFactorCamera < scalingFactorImu) {
        // We need to swap mode and reset orientation
        model = CAMERA_PASSTROUGH;
        // overwriteAttitude(m);
        ROS_INFO("Kalman filter motion model swapped to PASSTROUGH"); 
    }
    return getRollingModeActive();
}

/** ----------------------------------------------------
 * ----------------- Linear Kalman Filter --------------
 * -----------------------------------------------------*/

/* LKF state:
  (deltaX, (global)
   deltaY, (global)
   deltaZ, (global)
   deltaRoll, (local)
   deltaPitch, (local)
   deltaYaw, (local)
   w_x, (global)
   w_y, (global)
   w_z) (global)
*/
Eigen::VectorXf state = Eigen::VectorXf::Zero(9); 

// State propagation matrix F
Eigen::MatrixXf F(9, 9);

// control input u
Eigen::VectorXf control_input = Eigen::VectorXf::Zero(9);

// control input matrix
Eigen::MatrixXf G(9, 9);

// state covariance matrix
Eigen::MatrixXf P = 100*Eigen::MatrixXf::Identity(9, 9); //high number as initial value -> will convergate against smaller number

// measurement prediction
Eigen::VectorXf z = Eigen::VectorXf::Zero(9);

// measurement prediction matrix
Eigen::MatrixXf H = Eigen::MatrixXf::Identity(9, 9);

// process noise covariance matrix Q / System prediction noise
Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(9, 9);

// measurement noise covariance matrix
Eigen::MatrixXf R_imu(9, 9);
Eigen::MatrixXf R_cam(9, 9);

inline void useRollingMotionModel(const double dT)
{
    // Motion prediction matrix uses angular velocity for position
    // TODO: First 3 columns are the result of a cross product with CONSTANT ground normal (0, 0, -1)
    // implement input of the measured ground normal
    F <<0, 0, 0, 0, 0, 0, 0, -(dT * r_sphere), 0,
        0, 0, 0, 0, 0, 0, (dT * r_sphere), 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;

    // Control input matrix takes rotation and angular velocities
    G << 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
}

inline void useCameraMotionModel(const double dT)
{
    // Do no prediction at all
    F << 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;

    // Control input matrix takes the whole state
    G << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
}

// Prediction Step
void predictState(const double dT, Eigen::VectorXf u, Eigen::MatrixXf Q)
{
    // Switch the motion model to be used in KF
    if (getRollingModeActive()) {
        useRollingMotionModel(dT);
    } else {
        useCameraMotionModel(dT);
    }

    // Predict state (x_pri)
    state = F * state + G * u;   // 9x9 * 9x1 = 9x1
   
    // Calculate P_pri
    P = F * P * F.transpose() + Q;

    // Predict measurement
    z = H * state; 
}

// Update Step -> measurement comes from either imu or cam callback
void updateState(const Eigen::VectorXf &measurement, Eigen::MatrixXf R)
{
    // Innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // Calculate innovation
    Eigen::VectorXf innovation = measurement - z;
    
    // Update state estimation
    state = state + (K * innovation);
    
    // Calculate P_pos
    P = (Eigen::MatrixXf::Identity(9, 9) - K * H) * P;
}

// The magic function. Includes the interpolation of the faster stream.
void applyLkfAndPublish(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    /*
    * Interpolation of the faster stream, calculation in global frame 
    * will result in a Delta which applies in local frame.
    */

    // calculate dT
    double dT = (m->header.stamp - last_imu_pose.header.stamp).toSec();

    // Get timestamp from current and wait for accumulator to go past that
    ros::Time stamp_current = m->header.stamp;
    tfScalar t_current = stamp_current.toSec();

    // wait so that imu time stamp is inbetween cam time stamps
    waitForAccumulator(t_current); 

    /*
    * Interpolation of camera pose
    */ 

    // Convert geometry_msgs to Quaternion format
    tf::Quaternion q1, q2, q_res;
    tf::quaternionMsgToTF(accumulator.front().pose.pose.orientation, q1);
    tf::quaternionMsgToTF(accumulator.back().pose.pose.orientation, q2);

    // Convert geometry_msgs Points to tf Vectors
    tf::Vector3 v1, v2, v_res;
    tf::pointMsgToTF(accumulator.front().pose.pose.position, v1);
    tf::pointMsgToTF(accumulator.back().pose.pose.position, v2);

    // Get time parameter for slerp
    double t_acc_last = accumulator.front().header.stamp.toSec();
    double t_acc_latest = accumulator.back().header.stamp.toSec();
    double t = (t_current - t_acc_last) / (t_acc_latest - t_acc_last);

    // Interpolate rotation
    q_res = tf::slerp(q1, q2, t);
    // Interpolate position
    v_res = tf::lerp(v1, v2, t);
    // Construct interpolated result
    pose_interpolated = tf::Pose(q_res, v_res);

    // Rotate interpolated pose via basis change, into the frame of the faster stream
    pose_interpolated.mult(pose_interpolated, tf_axes_imu2cam);
    pose_interpolated.mult(tf_map_imu2cam, pose_interpolated);

    // interpolation camera angular velocities, which are given in LOCAL frame
    tf::Vector3 w1, w2, w_res;
    tf::vector3MsgToTF(accumulator.front().imu.angular_velocity, w1);
    tf::vector3MsgToTF(accumulator.back().imu.angular_velocity, w2);
    w_res = tf::lerp(w1, w2, t);

    // Rotate interpolated angular velocity via basis change into the frame of the faster stream
    tf::Pose tf_angular_vel_cam_pose;  // use tf::Pose to save angular velocity in x,y,z elements for multiplication operation
    tf_angular_vel_cam_pose.setOrigin(w_res);
    tf_angular_vel_cam_pose.setRotation(rot_zero);
    tf_angular_vel_cam_pose.mult(tf_angular_vel_cam_pose, tf_axes_imu2cam);
    tf_angular_vel_cam_pose.mult(tf_map_imu2cam, tf_angular_vel_cam_pose);
    
    // Angular velocities of IMU
    tf::Vector3 tf_angular_vel_imu;
    tf::vector3MsgToTF(m->imu.angular_velocity, tf_angular_vel_imu);
    
    /*
    * Rotate the local deltas into the global frame
    */

    // In first iteration, last interpolation does not exist. Use the first msg instead
    if (fabs(last_pose_interpolated.getRotation().length() - 1.0) > small_number) {
        ROS_WARN("Uninitialized quaternion, length: %f", last_pose_interpolated.getRotation().length());
        tf::Stamped<tf::Pose> last_pose_cam_stamped;
        tf::poseStampedMsgToTF(last_cam_pose.pose, last_pose_cam_stamped);
        last_pose_interpolated = last_pose_cam_stamped;
        last_pose_interpolated.mult(last_pose_interpolated, tf_axes_imu2cam);
        last_pose_interpolated.mult(tf_map_imu2cam, last_pose_interpolated);
        ROS_WARN("Attempted fix, length: %f", last_pose_interpolated.getRotation().length());
    }
    
    // Calculate delta of cam pose
    tf::Pose cam_diff_interpolated = pose_interpolated.inverseTimes(last_pose_interpolated);
    geometry_msgs::Pose cam_diff_geom_msgs;
    tf::poseTFToMsg(cam_diff_interpolated, cam_diff_geom_msgs);
    
    // Calculate delta of imu pose
    tf::Pose imu_diff = pose_diff(m, last_imu_pose);
    geometry_msgs::Pose imu_diff_geom_msgs;
    tf::poseTFToMsg(imu_diff, imu_diff_geom_msgs);
    
    // Approximate global rotation speed to decide rolling mode
    tf::Stamped<tf::Pose> current_pose;
    tf::poseStampedMsgToTF(m->pose, current_pose);
    tf::Transform current_tf_imu(current_pose.getRotation());
    tf::Vector3 current_vel_imu_rotated = current_tf_imu * tf_angular_vel_imu;
    decideRollingMode(current_vel_imu_rotated, m);

    // Construct internal transformations which we need to convert between local and global
    tf::poseStampedMsgToTF(filtered_pose_msg, current_pose);
    tf::Transform current_tf(current_pose.getRotation());

    // Make local deltas global
    tf::Pose imu_diff_rotated = current_tf * imu_diff;
    tf::Pose cam_diff_rotated = current_tf * cam_diff_interpolated;
    tf::Vector3 tf_angular_velocity_imu_rotated = current_tf * tf_angular_vel_imu;
    tf::Vector3 tf_angular_velocity_cam_rotated = current_tf * tf_angular_vel_cam_pose.getOrigin();

    // Put everything into the LKF process control vector
    Eigen::Vector3f eigen_angular_velocity_imu_rotated;
    eigen_angular_velocity_imu_rotated[0] = tf_angular_velocity_imu_rotated.getX();
    eigen_angular_velocity_imu_rotated[1] = tf_angular_velocity_imu_rotated.getY();
    eigen_angular_velocity_imu_rotated[2] = tf_angular_velocity_imu_rotated.getZ();

    Eigen::Vector3f eigen_angular_velocity_cam_rotated;
    eigen_angular_velocity_cam_rotated[0] = tf_angular_velocity_cam_rotated.getX();
    eigen_angular_velocity_cam_rotated[1] = tf_angular_velocity_cam_rotated.getY();
    eigen_angular_velocity_cam_rotated[2] = tf_angular_velocity_cam_rotated.getZ();
   
    // Interpolation camera tracker confidence and estimate covariance factors
    uint8_t pose_confidence_interpolated = (t * accumulator.front().tracker_confidence) 
        + ((1 - t) * accumulator.back().tracker_confidence);
    last_translated_confidence = (float) confidenceTranslator(pose_confidence_interpolated);
    scalingFactorCamera = getScalingFactor(tf_angular_velocity_imu_rotated, true);
    scalingFactorImu = getScalingFactor(tf_angular_velocity_imu_rotated, false);
    
    // Make vector 9dof for predict step
    Eigen::VectorXf eigen_input_rotated = Eigen::VectorXf::Zero(9);
    double dr, dp, dy;

    // Imu pose as control input
    if (getRollingModeActive()) {
        eigen_input_rotated[0] = imu_diff_rotated.getOrigin().getX();
        eigen_input_rotated[1] = imu_diff_rotated.getOrigin().getY();
        eigen_input_rotated[2] = imu_diff_rotated.getOrigin().getZ();
        quat2RPY(imu_diff_geom_msgs.orientation, dr, dp, dy);
        eigen_input_rotated[3] = dr;
        eigen_input_rotated[4] = dp;
        eigen_input_rotated[5] = dy;
        eigen_input_rotated[6] = eigen_angular_velocity_imu_rotated[0];
        eigen_input_rotated[7] = eigen_angular_velocity_imu_rotated[1];
        eigen_input_rotated[8] = eigen_angular_velocity_imu_rotated[2];
    // Cam pose as control input
    } else {
        eigen_input_rotated[0] = cam_diff_rotated.getOrigin().getX();
        eigen_input_rotated[1] = cam_diff_rotated.getOrigin().getY();
        eigen_input_rotated[2] = cam_diff_rotated.getOrigin().getZ();
        quat2RPY(cam_diff_geom_msgs.orientation, dr, dp, dy);
        eigen_input_rotated[3] = dr;
        eigen_input_rotated[4] = dp;
        eigen_input_rotated[5] = dy;
        eigen_input_rotated[6] = eigen_angular_velocity_cam_rotated[0];
        eigen_input_rotated[7] = eigen_angular_velocity_cam_rotated[1];
        eigen_input_rotated[8] = eigen_angular_velocity_cam_rotated[2];
    }
    
    /* 
     * Calculate Scaling factor for measurement noise covariance matrices
     + Take camera confidence and velocity into account
     + Low confidence -> higher variance -> more uncertainty
     + High angular velocity -> higher variance -> more uncertainty
     */
    
    Eigen::MatrixXf R_cam_scaled = R_cam * scalingFactorCamera;
    Eigen::MatrixXf R_imu_scaled = R_imu * scalingFactorImu;

    /*
     * Prediction step
     */

    predictState(dT, eigen_input_rotated, Q);

    /*
     * Update step, which is only necessary in ROLLING mode
     * Then, sensor fusion between IMU and CAM is done using two updates, with IMU data as control input.
     * If motion model is in PASSTROUGH mode, camera data is used as control input and update is skipped. 
     */

    if (getRollingModeActive()) {
        // update using IMU measurements
        quat2RPY(imu_diff_geom_msgs.orientation, dr, dp, dy);
        Eigen::VectorXf measurementImu(9);
        measurementImu << imu_diff_rotated.getOrigin().getX(), 
            imu_diff_rotated.getOrigin().getY(), 
            imu_diff_rotated.getOrigin().getZ(),
            dr, dp, dy,
            tf_angular_velocity_imu_rotated.getX(), 
            tf_angular_velocity_imu_rotated.getY(), 
            tf_angular_velocity_imu_rotated.getZ();
        updateState(measurementImu, R_imu_scaled);
        
        // update using CAM measurements
        quat2RPY(cam_diff_geom_msgs.orientation, dr, dp, dy);
        Eigen::VectorXf measurementCam(9);
        measurementCam << cam_diff_rotated.getOrigin().getX(), 
            cam_diff_rotated.getOrigin().getY(),
            cam_diff_rotated.getOrigin().getZ(),
            dr, dp, dy,
            tf_angular_velocity_cam_rotated.getX(),
            tf_angular_velocity_cam_rotated.getY(),
            tf_angular_velocity_cam_rotated.getZ();
        updateState(measurementCam, R_cam_scaled);
    }

    /*
     * Apply the LKF estimation to the filtered pose  
     */

    // The state contains the Deltas. Convert LKF state to delta tf
    tf::Pose filteredDeltaPose;
    filteredDeltaPose.setOrigin(current_tf.inverse() * tf::Vector3(state[0], state[1], state[2]));
    filteredDeltaPose.setRotation(tf::createQuaternionFromRPY(state[3], state[4], state[5]));
    
    // Apply the delta to the (last) current_pose
    tf::Stamped<tf::Pose> outputPose;
    tf::poseStampedMsgToTF(filtered_pose_msg, outputPose);
    current_pose.mult(current_pose, filteredDeltaPose.inverse()); // update der filtered pose
    tf::poseStampedTFToMsg(current_pose, filtered_pose_msg);      // update filtered_Pose_msg
    // TODO:(?) Reset attitude (except yaw) every iteration -> ugly
    // Rather look at the covariances to see what's going wrong 
    overwriteAttitude(m); 

    /*
     * Publish to topic
     */

    // Construct msg
    filtered_pose_msg.header.frame_id = "map";
    filtered_pose_msg.header.stamp = stamp_current;
    filtered_pose_msg.header.seq = sequence++;

    // publish filtered_pose_msg
    filtered_pose_pub.publish(filtered_pose_msg);

    // Prep next iteration
    last_pose_interpolated = pose_interpolated;
}

void imuMsgCallback(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_imu) {
        ROS_INFO("Kalman filter IMU odometry init");
        filtered_pose_msg = m->pose;
        last_imu_pose = *m;
        initialized_imu = true;
        return;
    }

    // If cam interpolation is active, wait for cam timestamps to go past current time and do interpolation
    if (interpolate == CAM) {
        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc) {
            // Use kalman filter and publish the pose msg
            applyLkfAndPublish(m);
        } 
    // Otherwise, if IMU interpolation is active, push current pose to queue
    } else if (interpolate == IMU) {
        pushToAccumulator(m);
    }

    // Save current pose to last pose for next iteration
    last_imu_pose = *m;
}

void camMsgCallback(const state_estimator_msgs::Estimator::ConstPtr &m)
{
    // Initialization on first run
    if (!initialized_cam) {
        ROS_INFO("Kalman filter visual-inertial odometry init");
        last_cam_pose = *m;
        pushToAccumulator(m);
        initialized_cam = true;
        return;
    }

    // If cam interpolation is active, push cam poses to queue
    if (interpolate == CAM) {
        pushToAccumulator(m);
    } else if (interpolate == IMU) {
        // Sanity check, can only interpolate if buffer is fully accumulated
        if (accumulator.size() == n_acc) {
            // Use kalman filter and publish the pose msg
            applyLkfAndPublish(m);
        }
    }

    // Debug msgs
    if (publish_debug_topic) {
        // Testing, apply correct rotation
        tf::Stamped<tf::Pose> this_pose;
        tf::poseStampedMsgToTF(m->pose, this_pose);
        tf::Pose rotated_pose;

        rotated_pose.mult(this_pose, tf_axes_imu2cam);
        rotated_pose.mult(tf_map_imu2cam, rotated_pose);

        debug_pose_msg.header = m->header;
        debug_pose_msg.header.frame_id = "map";
        tf::poseTFToMsg(rotated_pose, debug_pose_msg.pose);
        debug_pose_pub.publish(debug_pose_msg);
    }

    // Save current pose to last pose for next iteration
    last_cam_pose = *m;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_delta_filter_node");
    ros::NodeHandle nh;

    // Nodehandle for the faster stream (should be vector for more than 2 estimators)
    ros::NodeHandle nh_fast;
    nh_fast.setCallbackQueue(&callbacks_fast);

    // Topic params
    std::string topic_publish, topic_pose_imu, topic_pose_cam, topic_vel_imu;
    std::string frame_id_imu, frame_id_cam;
    nh.param<std::string>("topic_publish", topic_publish, "/lkf/pose");
    nh.param<std::string>("topic_pose_imu", topic_pose_imu, "/posePub_merged");
    nh.param<std::string>("topic_vel_imu", topic_vel_imu, "/orientation");
    nh.param<std::string>("topic_pose_cam", topic_pose_cam, "/camera/sphereCenterPose");
    nh.param<std::string>("frame_id_imu", frame_id_imu, "imu_frame");
    nh.param<std::string>("frame_id_cam", frame_id_cam, "camera_frame");
    nh.param<int>("imu_rate", imu_rate, 125); // Jaspers Code uses 125 Hz by default
    nh.param<int>("cam_rate", cam_rate, 200); // Intels T265 uses 200 Hz by default
    nh.param<double>("sphere_radius", r_sphere, 0.145);
    nh.param<double>("large_omega", large_omega, 0.5);
    nh.param<double>("small_omega", small_omega, 0.005);
    nh.param<bool>("debug_topics", publish_debug_topic, false);

    // Set omega thresholds
    large_omega2 = large_omega * large_omega;
    small_omega2 = small_omega * small_omega;

    // Determine accumulation window size of faster stream
    double imu_t = 1.0 / imu_rate, cam_t = 1.0 / cam_rate;
    if (imu_rate < cam_rate) {
        interpolate = CAM;
        n_acc = std::ceil(imu_t / cam_t) + 1;
        ROS_INFO("Interpolating Camera Pose measurements using %d samples", n_acc);
    } else if (cam_rate < imu_rate) {
        interpolate = IMU;
        n_acc = std::ceil(cam_t / imu_t) + 1;
        ROS_INFO("Interpolating Imu Pose measurements using %d samples", n_acc);
    } else {
        interpolate = NONE;
        n_acc = 1;
        ROS_INFO("Interpolation disabled.");
    }

    // Initialize variables
    initialized_cam = false;
    initialized_imu = false;

    // Get static tf between imu and camera frame
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("axes_cam", "axes_imu", ros::Time(0), ros::Duration(5.0));
    tf_listener.waitForTransform("map_imu", "map_cam", ros::Time(0), ros::Duration(5.0));
    tf_listener.lookupTransform("map_imu", "map_cam", ros::Time(0), tf_map_imu2cam);
    tf_listener.lookupTransform("axes_cam", "axes_imu", ros::Time(0), tf_axes_imu2cam);

    // Publishers and subscribers
    ros::Subscriber cam_pose_sub, imu_pose_sub, imu_vel_sub, cam_imu_vel_sub;
    if (interpolate == IMU) {
        cam_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh_fast.subscribe<state_estimator_msgs::Estimator>(topic_pose_imu, 1000, imuMsgCallback);
    } else if (interpolate == CAM) {
        cam_pose_sub = nh_fast.subscribe<state_estimator_msgs::Estimator>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_pose_imu, 1000, imuMsgCallback);
    } else if (interpolate == NONE) {
        cam_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_pose_cam, 1000, camMsgCallback);
        imu_pose_sub = nh.subscribe<state_estimator_msgs::Estimator>(topic_pose_imu, 1000, imuMsgCallback);
    }

    filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_publish, 1000);
    if (publish_debug_topic)
        debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/lkf/debug", 1000);

    // Main processing loop, wait for callbacks to happen
    fast_rate = std::max(cam_rate, imu_rate);
    slow_rate = std::min(cam_rate, imu_rate);
    node_frequency = std::make_shared<ros::Rate>(ros::Rate(1000));

    /*
    *  Initialise covariances
    */

    // IMU:
    Eigen::VectorXf imu_variances = Eigen::VectorXf::Zero(9); 
    
    // Order of entries:
    // [x, y, z, roll, pitch, yaw, angular_vel_x, angular_vel_y, angular_vel_z]^T
    
    // manually set
    imu_variances[0] = 0.01; 
    imu_variances[1] = 0.01; 
    imu_variances[2] = 0.01; 
    imu_variances[3] = 0.01;
    imu_variances[4] = 0.01; 
    imu_variances[5] = 0.08; 
    imu_variances[6] = 0.04; 
    imu_variances[7] = 0.04; 
    imu_variances[8] = 0.04;   
    R_imu << imu_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,
             0, imu_variances[1], 0, 0, 0, 0, 0, 0, 0,
             0, 0, imu_variances[2], 0, 0, 0, 0, 0, 0,
             0, 0, 0, imu_variances[3], 0, 0, 0, 0, 0,
             0, 0, 0, 0, imu_variances[4], 0, 0, 0, 0,
             0, 0, 0, 0, 0, imu_variances[5], 0, 0, 0,
             0, 0, 0, 0, 0, 0, imu_variances[6], 0, 0,
             0, 0, 0, 0, 0, 0, 0, imu_variances[7], 0,
             0, 0, 0, 0, 0, 0, 0, 0, imu_variances[8];

    // CAM:
    Eigen::VectorXf cam_variances = Eigen::VectorXf::Zero(9); 
    
    // manually set
    cam_variances[0] = 1;
    cam_variances[1] = 1;
    cam_variances[2] = 1;
    cam_variances[3] = 0.01;
    cam_variances[4] = 0.01;
    cam_variances[5] = 0.01;
    cam_variances[6] = 0.04;
    cam_variances[7] = 0.04;
    cam_variances[8] = 0.04;
    R_cam << cam_variances[0], 0, 0, 0, 0, 0, 0, 0, 0,
             0, cam_variances[1], 0, 0, 0, 0, 0, 0, 0,
             0, 0, cam_variances[2], 0, 0, 0, 0, 0, 0,
             0, 0, 0, cam_variances[3], 0, 0, 0, 0, 0,
             0, 0, 0, 0, cam_variances[4], 0, 0, 0, 0,
             0, 0, 0, 0, 0, cam_variances[5], 0, 0, 0,
             0, 0, 0, 0, 0, 0, cam_variances[6], 0, 0,
             0, 0, 0, 0, 0, 0, 0, cam_variances[7], 0,
             0, 0, 0, 0, 0, 0, 0, 0, cam_variances[8];

    // Init position
    filtered_pose_msg.pose.position.x = 0;
    filtered_pose_msg.pose.position.y = 0;
    filtered_pose_msg.pose.position.z = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        callbacks_fast.callOne();
        node_frequency->sleep();
    }

    return 0;
}
