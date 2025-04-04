#include "imuJasper.h"

// DEFINING EXTERN VARIABLES OF HEADER
// quiet mode
bool quiet = false;
// gain of madgwickfilter
float gain_ = DEFAULT_MADGWICK_GAIN;
// gain of complemnetary
float alpha = 0.02;
// Mechnaism for checking if everything works
int firstRead = 0;
// slow-mode ( (g_new = (g_old +g_0+ g_1 + g_2)/4))
bool slow = false;
// shoulr roll pitch yw in degrees be printed every loop
bool printRPY = false;
//
bool debugMode = false;
//
int data_rate = DATA_RATE_DEFAULT;
//
float data_intervall = 1 / DATA_RATE_DEFAULT;
//
int imu_data_rate = IMU_DATA_RATE_DEFAULT;
// gravity ing
float gravityValue = 1;
// automaticgain
float autogain = 0;
// float radius in m
float r = 0.145;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// n1 counts the number of samples gathered before the mean is taken of it (if) publishing rate is lower then imu rate
int n0 = 0;
int n1 = 0;
int n2 = 0;
std::mutex mtx_n; // protects the average
// name of topic
std::string topicName = "posePub_merged";
// determines if z should always be 0 (helps improving position if this assumption can be made))
bool setZ0 = false;

// just for evaluation
float q0_alt1 = 1;
float q1_alt1 = 0;
float q2_alt1 = 0;
float q3_alt1 = 0;
float q0_alt2 = 1;
float q1_alt2 = 0;
float q2_alt2 = 0;
float q3_alt2 = 0;
float q0_last_alt1 = 1;
float q1_last_alt1 = 0;
float q2_last_alt1 = 0;
float q3_last_alt1 = 0;
float q0_last_alt2 = 1;
float q1_last_alt2 = 0;
float q2_last_alt2 = 0;
float q3_last_alt2 = 0;

// Protects accelerometer access
std::mutex mtx_a0, mtx_a1, mtx_a2; 
float a0_alt[3] = {0, 0, 0};
float a1_alt[3] = {0, 0, 0};
float a2_alt[3] = {0, 0, 0};

float ax_alt = 0;
float ay_alt = 0;
float az_alt = 0;

int altStatus = 1;

float pXAlt = 0;
float pYAlt = 0;
float pZAlt = 0;

Lowpass low_pass;

void changeAlt()
{
    if (altStatus == 1) {
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
    } else {
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
};

// Handler for data from spatial0
void CCONV onSpatial0_SpatialData(PhidgetSpatialHandle ch, void *ctx, const double acceleration[3], const double angularRate[3], const double magneticField[3], double timestamp)
{
    int serialNr;
    int spatialNr;

    float *gxTemp;
    float *gyTemp;
    float *gzTemp;
    float *axTemp;
    float *ayTemp;
    float *azTemp;
    float dt_ms;

    Phidget_getDeviceSerialNumber((PhidgetHandle) ch, &serialNr);
    if (serialNr == SERIAL_0) {
        std::lock_guard<std::mutex> lock(mtx_s0);
        spatialNr = 0;
        gxTemp = &gx0;
        gyTemp = &gy0;
        gzTemp = &gz0;
        axTemp = &ax0;
        ayTemp = &ay0;
        azTemp = &az0;
        dt_ms = timestamp - lastTime_serial0_ms;
        lastTime_serial0_ms = timestamp;
        n0++;
    } else if (serialNr == SERIAL_1) {
        std::lock_guard<std::mutex> lock(mtx_s1);
        spatialNr = 1;
        gxTemp = &gx1;
        gyTemp = &gy1;
        gzTemp = &gz1;
        axTemp = &ax1;
        ayTemp = &ay1;
        azTemp = &az1;
        dt_ms = timestamp - lastTime_serial1_ms;
        lastTime_serial1_ms = timestamp;
        n1++;
    } else if (serialNr == SERIAL_2) {
        std::lock_guard<std::mutex> lock(mtx_s2);
        spatialNr = 2;
        gxTemp = &gx2;
        gyTemp = &gy2;
        gzTemp = &gz2;
        axTemp = &ax2;
        ayTemp = &ay2;
        azTemp = &az2;
        dt_ms = timestamp - lastTime_serial2_ms;
        lastTime_serial2_ms = timestamp;
        n2++;
    } else {
        ROS_WARN("Attention! IMU with serial %d was not defined in config.launch!", serialNr);
    }

    if (firstRead == spatialNr) {
        firstRead++;
        ROS_INFO("Receiving from IMU %d works! Serial: %d", spatialNr, serialNr);
    }

    float dt_s = dt_ms / 1000; // to seconds
    setValsAndCompensateCentripetal(serialNr, acceleration, angularRate, dt_s, gxTemp, gyTemp, gzTemp, axTemp, ayTemp, azTemp);
}

void setValsAndCompensateCentripetal(const int serialNr,
    const double acceleration[3], const double angularRate[3], const double dts,
    float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
{
    // Calc derivative of rotation speed
    *gx = angularRate[0] * M_PI_BY_180; // degrees to radians
    *gy = angularRate[1] * M_PI_BY_180;
    *gz = angularRate[2] * M_PI_BY_180;
    float gxl = gx_filtered * M_PI_BY_180, gyl = gy_filtered * M_PI_BY_180, gzl = gz_filtered * M_PI_BY_180;
    float w = sqrt(gxl * gxl + gyl * gyl + gzl * gzl);
    float wdot_noisy;
    wdot_noisy = (w - lastw) / dts;
    lastw = w;
    float wdot = low_pass.filter(wdot_noisy);

    // Set extrinsics based on serial number
    float *pos_serial;
    float *ur;
    float r;
    if (serialNr == SERIAL_0) {
        std::lock_guard<std::mutex> lock(mtx_a0);
        a0_alt[0] += acceleration[0];
        a0_alt[1] += acceleration[1];
        a0_alt[2] += acceleration[2];
        pos_serial = pos_serial0;
        r = r0;
        ur = ur0;
    }
    else if (serialNr == SERIAL_1) {
        std::lock_guard<std::mutex> lock(mtx_a1);
        a1_alt[0] += acceleration[0];
        a1_alt[1] += acceleration[1];
        a1_alt[2] += acceleration[2];
        pos_serial = pos_serial1;
        r = r1;
        ur = ur1;
    } else if (serialNr == SERIAL_2) {
        std::lock_guard<std::mutex> lock(mtx_a2);
        a2_alt[0] += acceleration[0];
        a2_alt[1] += acceleration[1];
        a2_alt[2] += acceleration[2];
        pos_serial = pos_serial2;
        r = r2;
        ur = ur2;
    } else {
        ROS_ERROR("IMU unexpected error: Serial %d not found.", serialNr);
    }

    // Compensate radial and tangential accelerations due to centripetal force caused by rotation
    float g[3] = {gxl, gyl, gzl};
    float utheta[3];
    vectorAsNormalized(g, utheta);
    *ax += acceleration[0] + r * w * w * ur[0] - r * wdot * (utheta[1] * ur[2] - utheta[2] * ur[1]);
    *ay += acceleration[1] + r * w * w * ur[1] - r * wdot * (utheta[2] * ur[0] - utheta[0] * ur[2]);
    *az += acceleration[2] + r * w * w * ur[2] - r * wdot * (utheta[0] * ur[1] - utheta[1] * ur[0]);
}

// we have to invert the gyro data as imus using not the rigth handed conversion but left handed.
// thefore the y axis is inverted (leading to gy not having to be converted)
void combineRAWData()
{   
    int n = 0;
    n = n0 + n1 + n2;
    
    /* HANDLE ACCELEROMETER */
    // Mean of accelerometers
    if (n != 0) {
        std::lock_guard<std::mutex> lock(mtx_n);
        ax = (-ax0 - ax1 - ax2) / n;
        ay = (-ay0 - ay1 - ay2) / n;
        az = (-az0 - az1 - az2) / n;
        ax_alt = (-a0_alt[0] - a1_alt[0] - a2_alt[0]) / n;
        ay_alt = (-a0_alt[1] - a1_alt[1] - a2_alt[1]) / n;
        az_alt = (-a0_alt[2] - a1_alt[2] - a2_alt[2]) / n;
        a0_alt[0] = 0; a0_alt[1] = 0; a0_alt[2] = 0;
        a1_alt[0] = 0; a1_alt[1] = 0; a1_alt[2] = 0;
        a2_alt[0] = 0; a2_alt[1] = 0; a2_alt[2] = 0;
        n0 = 0; n1 = 0; n2 = 0;
        ax0 = 0; ay0 = 0; az0 = 0;
        ax1 = 0; ay1 = 0; az1 = 0;
        ax2 = 0; ay2 = 0; az2 = 0;
    }

    /* HANDLE GYROSCOPE */
    if (firstRead < 4) {
        ROS_INFO("Init Yaw Axis...");
        gx = (gx1 + gx2 + gx0) * ONE_THIRD;
        gy = (gy1 + gy2 + gy0) * ONE_THIRD;
        gz = (gz1 + gz2 + gz0) * ONE_THIRD;
        ovrwrtOrientWithAcc(ax, ay, az, 0);
        changeAlt();
        ovrwrtOrientWithAcc(ax, ay, az, 0);
        changeAlt();
        firstRead = 4;
    } else if (!slow) /* THIS IS THE USUAL CASE ! */ {
        gx = (gx1 + gx2 + gx0) * ONE_THIRD;
        gy = (gy1 + gy2 + gy0) * ONE_THIRD;
        gz = (gz1 + gz2 + gz0) * ONE_THIRD;
    } else {
        gx = (gx1 + gx2 + gx0 + gx) * 0.25;
        gy = (gy1 + gy2 + gy0 + gy) * 0.25;
        gz = (gz1 + gz2 + gz0 + gz) * 0.25;
    }
}

void madgwick_and_complementary_Filter()
{

    ros::Time tmp = ros::Time::now();
    dt = (tmp.toNSec() - lastTime.toNSec()) * 0.000000001;
    lastTime = tmp;

    // if dt is 10% longer then it should be, we consider Lacking
    if (dt > data_intervall * 1.1) {
        ROS_WARN("Phidgets IMUs lagging behind");
    }
    if (dt <= data_intervall * 0.1) {
        //ROS_WARN("Phidgets IMUs jittering!");
        dt = data_intervall; // TODO: does this make sense??
    }

    // Helper variables
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // calculating the factors describign if there is active rotation.
    float factorX = (0.5 * gx) * (0.5 * gx);
    float factorY = (0.5 * gy) * (0.5 * gy);
    float factorZ = (0.5 * gz) * (0.5 * gz);
    if (factorX > 1)
        factorX = 1;
    if (factorY > 1)
        factorY = 1;
    if (factorZ > 1)
        factorZ = 1;

    // adapt gains automatically
    if (!autogain <= 0)
    {
        float maxFac = std::max(std::max(factorY, factorZ), factorX);
        // alpha is one magnitude less, as the standard values for both are 0.2 and 0.02
        gain_ = autogain * maxFac;
        gain_ = std::max(gain_, gain_min);
        if (maxFac < 0.1)
            maxFac = 0;
        alpha = autogain * 0.1 * (1 - maxFac);
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = 1;
    }
    else
    {
        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    }
    float axNom = ax * recipNorm;
    float ayNom = ay * recipNorm;
    float azNom = az * recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * axNom + _4q0 * q1q1 - _2q1 * ayNom;
    s1 = _4q1 * q3q3 - _2q3 * axNom + 4.0f * q0q0 * q1 - _2q0 * ayNom - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * azNom;
    s2 = 4.0f * q0q0 * q2 + _2q0 * axNom + _4q2 * q3q3 - _2q3 * ayNom - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * azNom;
    s3 = 4.0f * q1q1 * q3 - _2q1 * axNom + 4.0f * q2q2 * q3 - _2q2 * ayNom;
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= gain_ * s0;
    qDot2 -= gain_ * s1;
    qDot3 -= gain_ * s2;
    qDot4 -= gain_ * s3;

    // save current quaternion as last orientation for next iteration
    lastq0 = q0;
    lastq1 = q1;
    lastq2 = q2;
    lastq3 = q3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    normalizeQuat(&q0, &q1, &q2, &q3);

    // Complementary Filter
    float acc_roll = atan2f(ayNom, azNom);
    float acc_pitch = atan2f(-axNom, sqrt(ayNom * ayNom + azNom * azNom));

    // get RPY from quaternion

    // acc_roll= atan(copysign(1.0,az)*ay/sqrt(0.01*ax*ax+az*az));
    float roll, pitch, yaw;
    eulerFromQuat(&roll, &pitch, &yaw, q0, q1, q2, q3);

    // the actual complementary step

    // gimbal lock avoidence
    if (fabs(q2 * q1 + q0 * q3) - 0.5 < 0.01)
    {
        acc_roll = roll;
    }

    float q00, q11, q22, q33;
    quatFromEuler(&q00, &q11, &q22, &q33, acc_roll, acc_pitch, yaw);

    quaternion qAcc, qCur, qNex;

    qAcc.w = q00;
    qAcc.x = q11;
    qAcc.y = q22;
    qAcc.z = q33;

    qCur.w = q0;
    qCur.x = q1;
    qCur.y = q2;
    qCur.z = q3;

    quaternion_slerp(&qCur, &qAcc, &qNex, alpha);

    q0 = qNex.w;
    q1 = qNex.x;
    q2 = qNex.y;
    q3 = qNex.z;

    normalizeQuat(&q0, &q1, &q2, &q3);
    eulerFromQuat(&roll, &pitch, &yaw, q0, q1, q2, q3);
    
    // Calculate filtered rot speed using quaternions to axisangle
    // Get previous orientation and invert it, in order to calc filtered rot speed later
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
    
    // calcualte filtered rotation speed
    gx_filtered = axis[0] * angle;
    gy_filtered = axis[1] * angle;
    gz_filtered = axis[2] * angle;

    //----- jasper filter starts here --------
    if (autogain == 0) return;

    // rotation matrix (from robot to world)

    // First row of the rotation matrix
    float r00 = 1 - 2 * (q2 * q2 + q3 * q3);
    float r01 = 2 * (q1 * q2 - q0 * q3);
    float r02 = 2 * (q1 * q3 + q0 * q2);

    // Second row of the rotation matrix
    float r10 = 2 * (q1 * q2 + q0 * q3);
    float r11 = 1 - 2 * (q1 * q1 + q3 * q3);
    float r12 = 2 * (q2 * q3 - q0 * q1);

    // Third row of the rotation matrix
    float r20 = 2 * (q1 * q3 - q0 * q2);
    float r21 = 2 * (q2 * q3 + q0 * q1);
    float r22 = 1 - 2 * (q1 * q1 + q2 * q2);

    // gravitational vector : rotate 001 from world ointo robot
    float grav_x = r20;
    float grav_y = r21;
    float grav_z = r22;

    if (fabs(ax - grav_x) > 0.02)
        vel_x += (ax - grav_x) * 9.81 * dt;
    if (fabs(ay - grav_y) > 0.02)
        vel_y += (ay - grav_y) * 9.81 * dt;
    if (fabs(az - grav_z) > 0.02)
        vel_z += (az - grav_z) * 9.81 * dt;
    
    // couple it to the rotation speed!
    // rotation around x enables translation-verlocity in y and z direction (robot frame)
    vel_x *= std::max(factorY, factorZ);
    vel_y *= std::max(factorZ, factorX);
    vel_z *= std::max(factorX, factorY);

    // integrate velocity to the points an rotate into world frame
    // use this matrix tranformed to get back rotation
    float vel_x_world = r00 * vel_x + r01 * vel_y + r02 * vel_z;
    float vel_y_world = r10 * vel_x + r11 * vel_y + r12 * vel_z;
    float vel_z_world = r20 * vel_x + r21 * vel_y + r22 * vel_z;
    if (setZ0) vel_z_world = 0;

    // rotation leads to translation (without slippage)
    float rot_x_world = r00 * gx_filtered + r01 * gy_filtered + r02 * gz_filtered;
    float rot_y_world = r10 * gx_filtered + r11 * gy_filtered + r12 * gz_filtered;
    float vel_x_world_rot = rot_y_world * r;
    float vel_y_world_rot = -rot_x_world * r;

    // only evaluation (use unfiltered rotation speed from gyro directly)
    float rot_x_world_alt = r00 * gx + r01 * gy + r02 * gz;
    float rot_y_world_alt = r10 * gx + r11 * gy + r12 * gz;
    float vel_x_world_rot_alt = rot_y_world_alt * r;
    float vel_y_world_rot_alt = -rot_x_world_alt * r;
    pXAlt += vel_x_world_rot_alt * dt;
    pYAlt += vel_y_world_rot_alt * dt;

    // limit velocity_z to the mean og vel_x by to and vel_y y rot. As Otherwise exponential error integration could happen.
    float mean_vel_world_XY = sqrt(vel_x_world_rot * vel_x_world_rot + vel_y_world_rot * vel_y_world_rot);
    if (fabs(vel_z_world) > mean_vel_world_XY)
        vel_z_world = std::copysign(1.0, vel_z_world) * mean_vel_world_XY;

    // to prevent uncontrollable expoential error interation when rolling over a long time in one direction 
    // (the factor_x for exapmple is 1, thefore the original problem with accellerometer integration occurs)
    // we limit the velocity to 110% of the corressponending velocity by rotation.
    // we skip z coordinate due to flat floor assumption
    float trust = 0.1;
    if (fabs(vel_x_world) > fabs(vel_x_world_rot))
        vel_x_world = std::min((1 + trust) * vel_x_world_rot, vel_x_world);
    if (fabs(vel_y_world) > fabs(vel_y_world_rot))
        vel_y_world = std::min((1 + trust) * vel_y_world_rot, vel_y_world);
    
    // subtract velocity in z (othwerwie we would roll through the obstacle rather then over it). 
    // if vz^2 is bigger then the rot velocity, we should take 0. 
    // because it somehow means we are falling (mor translation in z than possible by rotation). 
    // The square method takes care of this.
    float vz2 = vel_z_world * vel_z_world;
    vel_x_world_rot = std::copysign(1.0, vel_x_world_rot) * sqrt(vel_x_world_rot * vel_x_world_rot - vz2);
    vel_y_world_rot = std::copysign(1.0, vel_y_world_rot) * sqrt(vel_y_world_rot * vel_y_world_rot - vz2);
    
    if (sqrt(vel_x_world_rot*vel_x_world_rot + vel_y_world_rot * vel_y_world_rot) > 0.001) {
        px += vel_x_world_rot * dt;
        py += vel_y_world_rot * dt;
        pz = 0;
    }
}

int argumentHandler(ros::NodeHandle &nh)
{
    nh.param<bool>("jasper_quiet", quiet, true);
    nh.param<float>("jasper_gain", gain_, 0);
    gain_min = gain_;
    nh.param<float>("jasper_alpha", alpha, 0.02);
    nh.param<float>("jasper_autogain", autogain, 0);
    if (autogain > 0) {
        alpha = autogain;
        ROS_INFO("autogain set to %f\n. Thefore gain_ is set to 0 and alpha initally to %f", autogain, autogain * 0.1);
    }
    nh.param<bool>("jasper_slow", slow, false);
    int temp;
    nh.param<int>("jasper_pub_rate", temp, 125);
    if (temp > 0 && temp < 501) {
        data_rate = temp;
        data_intervall = 1.0 / temp;
        ROS_INFO("data rate set to %d\n", data_rate);
    } else {
        ROS_WARN("Phidgets IMU data rate allowed between 1 and 500 Hz. Using default at %d", DATA_RATE_DEFAULT);
    }
    nh.param<bool>("jasper_printRPY", printRPY, false);
    nh.param<int>("jasper_imu_rate", imu_data_rate, 250);
    imu_data_rate = imu_data_rate > 250 ? 250 : imu_data_rate;
    nh.param<bool>("jasper_debug", debugMode, false);
    nh.param<std::string>("topicName", topicName, std::string("posePub_merged"));
    nh.param<bool>("jasper_forceflat", setZ0, true);
    nh.param<int>("jasper_serial0", SERIAL_0, 0);
    nh.param<int>("jasper_serial1", SERIAL_1, 0);
    nh.param<int>("jasper_serial2", SERIAL_2, 0);
    /*
    The extrinsic parameters are defined using the coordinate system of the IMU.
    That is, in the config file, you must specify the vector to the center point of the ball, as seen from
    each individual IMU.
    In the later calculations, we need the inverse vector however, pointing FROM the center TO the IMU.
    That is why the signs are changed. Otherwise, the cross product yields the opposite direction.
    */
    float tmpx, tmpy, tmpz;
    nh.param<float>("imu0_x", tmpx, 0);
    nh.param<float>("imu0_y", tmpy, 0);
    nh.param<float>("imu0_z", tmpz, 0);
    pos_serial0 = new float[3]{-tmpx, -tmpy, -tmpz};
    r0 = sqrt(tmpx * tmpx + tmpy * tmpy + tmpz * tmpz);
    ur0 = new float[3];
    vectorAsNormalized(pos_serial0, ur0);
    nh.param<float>("imu1_x", tmpx, 0);
    nh.param<float>("imu1_y", tmpy, 0);
    nh.param<float>("imu1_z", tmpz, 0);
    pos_serial1 = new float[3]{-tmpx, -tmpy, -tmpz};
    r1 = sqrt(tmpx * tmpx + tmpy * tmpy + tmpz * tmpz);
    ur1 = new float[3];
    vectorAsNormalized(pos_serial1, ur1);
    nh.param<float>("imu2_x", tmpx, 0);
    nh.param<float>("imu2_y", tmpy, 0);
    nh.param<float>("imu2_z", tmpz, 0);
    pos_serial2 = new float[3]{-tmpx, -tmpy, -tmpz};
    r2 = sqrt(tmpx * tmpx + tmpy * tmpy + tmpz * tmpz);
    ur2 = new float[3];
    vectorAsNormalized(pos_serial2, ur2);
    float freq_cut;
    nh.param<float>("jasper_lowpass_freq", freq_cut, 10);
    low_pass.setFreq(freq_cut, imu_data_rate);
    return 0;
}

void CCONV detachHandler(PhidgetHandle ch, void *ctx)
{
    int serialNr;
    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &serialNr);
    ROS_ERROR("IMU with Serial Nr: %d detached!!!", serialNr);
}

int CCONV init()
{
    ROS_INFO("Initalizing");
    Phidget_resetLibrary();
    PhidgetSpatialHandle spatial0, spatial1, spatial2;
    
    // Create your Phidget channels
    PhidgetSpatial_create(&spatial0);
    PhidgetSpatial_create(&spatial1);
    PhidgetSpatial_create(&spatial2);
    
    // Set addressing parameters to specify which channel to open (if any)
    Phidget_setDeviceSerialNumber((PhidgetHandle) spatial0, SERIAL_0);
    Phidget_setDeviceSerialNumber((PhidgetHandle) spatial1, SERIAL_1);
    Phidget_setDeviceSerialNumber((PhidgetHandle) spatial2, SERIAL_2);
    
    // set the handler which handels detaching evcents
    Phidget_setOnDetachHandler((PhidgetHandle) spatial0, detachHandler, NULL);
    Phidget_setOnDetachHandler((PhidgetHandle) spatial1, detachHandler, NULL);
    Phidget_setOnDetachHandler((PhidgetHandle) spatial2, detachHandler, NULL);
    
    ROS_INFO("Now Attaching the IMUS! Giving it maximum 3 seconds!");
    // Open your Phidgets and wait for attachment
    ros::Time begin = ros::Time::now();
    Phidget_openWaitForAttachment((PhidgetHandle) spatial0, 1000);
    Phidget_openWaitForAttachment((PhidgetHandle) spatial1, 1000);
    Phidget_openWaitForAttachment((PhidgetHandle) spatial2, 1000);
    if (ros::Time::now().toSec() - begin.toSec() >= 3) {
        ROS_WARN("Phidgets IMU attachment timeout! Data may be compromised.");
    } else {
        ROS_INFO("Phidgets IMU attaching sucessfull.");
    }
    
    // Set the data rate (set by user or IMU_DEFAULT_DATA_RATE
    int imu_data_intervall = std::round(1000.0 / imu_data_rate);
    PhidgetSpatial_setDataInterval(spatial0, imu_data_intervall);
    PhidgetSpatial_setDataInterval(spatial1, imu_data_intervall);
    PhidgetSpatial_setDataInterval(spatial2, imu_data_intervall);
    
    ROS_WARN("!!! CALIBRATING GYROSCOPES, DO NOT MOVE IMUs !!!");
    ros::Duration(0.5).sleep();
    PhidgetSpatial_zeroGyro(spatial0);
    PhidgetSpatial_zeroGyro(spatial1);
    PhidgetSpatial_zeroGyro(spatial2);
    PhidgetSpatial_zeroAlgorithm(spatial0);
    PhidgetSpatial_zeroAlgorithm(spatial1);
    PhidgetSpatial_zeroAlgorithm(spatial2);
    ROS_WARN("Gyroscope calibration succesfull!");
    
    // Assign any event handlers you need before calling open so that no events are missed.
    PhidgetSpatial_setOnSpatialDataHandler(spatial0, onSpatial0_SpatialData, NULL);
    PhidgetSpatial_setOnSpatialDataHandler(spatial1, onSpatial0_SpatialData, NULL);
    PhidgetSpatial_setOnSpatialDataHandler(spatial2, onSpatial0_SpatialData, NULL);
    
    // initilizind data for filtering
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    vel_x = 0;
    vel_y = 0;
    vel_z = 0;
    
    // initilaize output message
    seq = 0;
    output_msg.header.frame_id = "map";
    output_msg.header.seq = seq++;
    output_msg.header.stamp = ros::Time::now();
    output_msg.pose.pose.position.x = 0;
    output_msg.pose.pose.position.y = 0;
    output_msg.pose.pose.position.z = 0;
    output_msg.pose.pose.orientation.x = q1;
    output_msg.pose.pose.orientation.y = q2;
    output_msg.pose.pose.orientation.z = q3;
    output_msg.pose.pose.orientation.w = q0;
    ROS_INFO("Phidgets initalisation done");
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imuHandling", ros::init_options::NoSigintHandler);
    ROS_INFO("Starting Node");
    if (argc > 1) {
        ROS_INFO("Command-line args are not supported anymore.");
        ROS_INFO("Please use the ROS parameter server and have a look at the config.launch file!");
    }
    signal(SIGINT, mySigIntHandler);

    // Arguments as nodehandle parameters (from ROS server)
    ros::NodeHandle nH;
    argumentHandler(nH);
    init();

    ros::Publisher estimator_pub = nH.advertise<state_estimator_msgs::Estimator>(topicName, 1000);
    //---------------------------------------
    // just for evaluation
    ros::Publisher estimator_raw_pub = nH.advertise<state_estimator_msgs::Estimator>(topicName + "_raw", 1000);
    //-------------------------------------

    ros::Rate loop_rate(data_rate);

    // wait for every imu to send data
    while (firstRead < 3 && !g_request_shutdown) {
        ROS_WARN("Waiting for all IMUs to deliever data...");
        ros::Rate loop_rate_StartUp(1);
        loop_rate_StartUp.sleep();
    }
    // the gyro send right after starting just the value  0 for 1-3 seconds. We dont need that
    while (gx0 == 0 && gx1 == 0 && gx2 == 0 && !g_request_shutdown) {
        ROS_WARN("Phidgets gyroscopes still waiting...");
        ros::Rate loop_rate_StartUp(1);
        loop_rate_StartUp.sleep();
    }
    lastTime = ros::Time::now();

    ROS_INFO("Phidigets IMUs are now properly initialized!");
    while (ros::ok() && !g_request_shutdown) {
        // Each cycle gathers data and filters
        combineRAWData();
        madgwick_and_complementary_Filter();

        // Fill msg header
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "map";
        output_msg.header.seq = seq++;
        
        // Position determined by IMUs
        output_msg.pose.header = output_msg.header;
        output_msg.pose.pose.position.x = px;
        output_msg.pose.pose.position.y = py;
        output_msg.pose.pose.position.z = pz;
        output_msg.pose.pose.orientation.w = q0;
        output_msg.pose.pose.orientation.x = q1;
        output_msg.pose.pose.orientation.y = q2;
        output_msg.pose.pose.orientation.z = q3;
        
        // Raw Data of IMUs
        output_msg.imu.header = output_msg.header;
        output_msg.imu.orientation = output_msg.pose.pose.orientation;
        // Gyroscope data is actually orientation differential 
        output_msg.imu.angular_velocity.x = gx_filtered;
        output_msg.imu.angular_velocity.y = gy_filtered;
        output_msg.imu.angular_velocity.z = gz_filtered;
        // Acceleration data compensates centripetal forces
        output_msg.imu.linear_acceleration.x = ax;
        output_msg.imu.linear_acceleration.y = ay;
        output_msg.imu.linear_acceleration.z = az;
        estimator_pub.publish(output_msg);

        //-------------------------------------//
        // just for evaluation purposes
        // change to alternative flawed values which uses unfiltered gyro data for position,
        // and also does not apply centripetal force compensation
        changeAlt();
        ax = ax_alt;
        ay = ay_alt;
        az = az_alt;

        // Use alternative data and filter it
        madgwick_and_complementary_Filter();

        // Put filtered data inside msg
        output_msg.pose.pose.position.x = pXAlt;
        output_msg.pose.pose.position.y = pYAlt;
        output_msg.pose.pose.position.z = pZAlt;
        output_msg.pose.pose.orientation.w = q0;
        output_msg.pose.pose.orientation.x = q1;
        output_msg.pose.pose.orientation.y = q2;
        output_msg.pose.pose.orientation.z = q3;
        output_msg.imu.orientation = output_msg.pose.pose.orientation;
        // RAW gyroscope data
        output_msg.imu.angular_velocity.x = gx;
        output_msg.imu.angular_velocity.y = gy;
        output_msg.imu.angular_velocity.z = gz;
        // RAW accelerometer data
        output_msg.imu.linear_acceleration.x = ax;
        output_msg.imu.linear_acceleration.y = ay;
        output_msg.imu.linear_acceleration.z = az;
        estimator_raw_pub.publish(output_msg);
        changeAlt();
        //-------------------------------------------------

        ros::spinOnce();
        loop_rate.sleep();
    } // end main loop

    ROS_WARN("Phidgets IMUs shutdown, GOODBYE!");
    Phidget_resetLibrary();
    ros::shutdown();
}

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw)
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

void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ)
{
    *roll = atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY);
    *pitch = asinf(-2.0f * (qX * qZ - qW * qY));
    *yaw = atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);
}

inline void normalizeQuat(float *q0, float *q1, float *q2, float *q3)
{
    float recipNorm = 1.0 / sqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;
}

inline void vectorAsNormalized(const float *v, float* res)
{
    float v2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    float recipNorm = v2 < 0.0001 ? 0 : 1.0 / sqrt(v2);
    res[0] = v[0] * recipNorm;
    res[1] = v[1] * recipNorm;
    res[2] = v[2] * recipNorm;
}

void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw)
{
    float recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    float acc_roll = atan2(ay, az);
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    quatFromEuler(&q0, &q1, &q2, &q3, acc_roll, acc_pitch, yaw);
    ROS_INFO("Initial: Roll: %f Pitch: %f Yaw: %f", acc_roll * precalc_180_BY_M_PI, acc_pitch * precalc_180_BY_M_PI, yaw * precalc_180_BY_M_PI);
}
void quaternion_slerp(quaternion *l, quaternion *r, quaternion *o, double weight)
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
double quaternion_dot_product(quaternion *l, quaternion *r)
{
    return l->w * r->w + l->x * r->x + l->y * r->y + l->z * r->z;
}

void quaternion_normalize(quaternion *q)
{
    double norm = 1.0 / sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0)
        norm = 0.0000001;
    q->w *= norm;
    q->x *= norm;
    q->y *= norm;
    q->z *= norm;
}
void quaternion_copy(quaternion *original_q, quaternion *copy_q)
{
    copy_q->w = original_q->w;
    copy_q->x = original_q->x;
    copy_q->y = original_q->y;
    copy_q->z = original_q->z;
}

void quaternion_inverse(const quaternion* q, quaternion* res)
{
    res->w =  q->w;
    res->x = -q->x;
    res->y = -q->y;
    res->z = -q->z;
    quaternion_normalize(res);
}

void quaternion_multiply(const quaternion* q1, const quaternion* q2, quaternion* res)
{
    res->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    res->x = q1->w * q2->x + q1->x * q2->w - q1->y * q2->z + q1->z * q2->y;
    res->y = q1->w * q2->y + q1->x * q2->z + q1->y * q2->w - q1->z * q2->x;
    res->z = q1->w * q2->z - q1->x * q2->y + q1->y * q2->x + q1->z * q2->w;
    quaternion_normalize(res);
}

void quaternion_to_axis_angle(const quaternion* in, double* axis, double* angle) 
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

