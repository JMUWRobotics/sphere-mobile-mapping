#ifndef IMU_MERGER_H
#define IMU_MERGER_H

// System
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <mutex>

// Phidget
#include <phidget22.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <state_estimator_msgs/Estimator.h>

#define DATA_RATE_DEFAULT 250
#define IMU_DATA_RATE_DEFAULT 250

#define DEFAULT_MADGWICK_GAIN 0.1

//often used calculations
#define M_PI_BY_180 0.017453292
#define precalc_180_BY_M_PI 57.29577951
#define ONE_THIRD 0.333333333

// NON-CONST Serial numbers of the IMUs
int SERIAL_0, SERIAL_1, SERIAL_2;

// Protects each IMU 
std::mutex mtx_s0, mtx_s1, mtx_s2;

// Extrinsics of each IMU wrt the center point of the sphere
float *pos_serial0;
float *pos_serial1;
float *pos_serial2;
// Same extrinsics as normalized unit vectors
float *ur0;
float *ur1;
float *ur2;
// Length of the extrinsic vectors = Radius of rotating lever arm
float r0, r1, r2;

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

state_estimator_msgs::Estimator output_msg;
int seq; // sequence number that counts number of msgs

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

ros::Time lastTime;
double lastTime_serial0_ms;
double lastTime_serial1_ms;
double lastTime_serial2_ms;
float lastw;

//Functions
//initalizing procedure
int CCONV init();
//handlign arguments giving to programm
int argumentHandler(ros::NodeHandle &nh);
//main filter function. @param reuse_dt: Set true if filter should be applied with alternative input values
void madgwick_and_complementary_Filter(bool reuse_dt = false);
// Data assosciacion and centripetal compensation
void setValsAndCompensateCentripetal(const int serialNr,
    const double acceleration[3], const double angularRate[3], const double dts,
    float *gx, float *gy, float *gz, float *ax, float *ay, float *az); 

//for safe shutdown
 void mySigIntHandler(int sig);
// calculate qauternion an write it into the qW to qZ
void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw);
//calculate euler an write it into yaw pitch roll
void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ);
//normalize quaternion values inplace
inline void normalizeQuat(float *q0, float *q1, float *q2, float *q3);
inline void vectorAsNormalized(const float* v, float* res);
//ovewrite q0-q3 with an estimation of accelerometer and the desired yaw (can not be set from accelerometer
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw);

// Implements a low pass filter as an exponential moving average
// See https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/Exponential-Moving-Average.html
// for details on why this is equivalent, i.e., alpha = sqrt(c*c - 4*c + 3) + c - 1; 
class Lowpass {
    private: 
        float smoothed;
        float alpha;
    public:
        Lowpass() {};
        float filter(float in) {
            smoothed = smoothed - alpha*(smoothed-in);
            return smoothed;
        }
        void setFreq(float f_cut, float f_sample) {
            float c = cos(2 * M_PI * f_cut / f_sample);
            alpha = sqrt(c*c - 4*c + 3) + c - 1; 
        }
};
#endif
