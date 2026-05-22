/**
 * @author Fabian Arzberger, JMU
 * @author Jasper Zevering, JMU
 *
 * Sensor-acquisition node.
 *
 * This node:
 *   1. Opens up to three Phidget Spatial IMUs by serial number.
 *   2. Receives raw acceleration (g) and angular rate (deg/s) callbacks.
 *   3. Converts units and applies per-IMU intrinsic calibration
 *      (bias, diagonal scale, misalignment matrix — loaded from YAML).
 *   4. Synchronises Phidget timestamps to the ROS clock.
 *   5. Publishes per-IMU raw and calibrated sensor_msgs/Imu messages on:
 *        /imu/0/raw    /imu/1/raw    /imu/2/raw
 *        /imu/0/calib  /imu/1/calib  /imu/2/calib
 *
 * What this node does NOT do:
 *   - Centripetal / lever-arm compensation  (→ filter.cpp)
 *   - Multi-IMU averaging                  (→ filter.cpp)
 *   - Attitude estimation / filtering       (→ filter.cpp)
 *   - Position / odometry computation       (→ filter.cpp)
 *   - Pose publishing                       (→ filter.cpp)
 */
#include "imuJasper.hpp"

void apply_intrinsics(
    const double raw[3],
    std::vector<double> &align,
    std::vector<double> &scale,
    std::vector<double> &bias,
    double *result)
{
    // Apply bias first
    result[0] = raw[0] + bias[0];
    result[1] = raw[1] + bias[1];
    result[2] = raw[2] + bias[2];

    // Apply scale
    double tmp0 = scale[0] * result[0];
    double tmp1 = scale[4] * result[1];
    double tmp2 = scale[8] * result[2];

    // Apply misalignment matrix
    result[0] = align[0] * tmp0 + align[1] * tmp1 + align[2] * tmp2;
    result[1] = align[3] * tmp0 + align[4] * tmp1 + align[5] * tmp2;
    result[2] = align[6] * tmp0 + align[7] * tmp1 + align[8] * tmp2;
}

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
    ros::Publisher &calibrated_pub)
{
    // Synchronise Phidget timestamp to ROS clock
    timestamp = sync.online_sync(timestamp, ros_now_s * 1000.0);
    last_time_ms = timestamp;

    // Apply intrinsic calibration
    apply_intrinsics(acc_inverse, acc_align, acc_scale, acc_bias, acc_calibrated);
    apply_intrinsics(angular_radians, gyr_align, gyr_scale, gyr_bias, gyr_calibrated);

    // centripetal compensation now done in filter.cpp

    /*
     * -- Construct the calibrated msg --
     */
    ms_to_ros_stamp(timestamp, imu_msg.header.stamp);
    imu_msg.header.frame_id = std::string(frame);
    imu_msg.header.seq = count++;

    imu_msg.angular_velocity.x = gyr_calibrated[0];
    imu_msg.angular_velocity.y = gyr_calibrated[1];
    imu_msg.angular_velocity.z = gyr_calibrated[2];
    // Publish the calibrated (but not centripetal-compensated) acceleration.
    // filter.cpp will apply centripetal compensation on its side.
    imu_msg.linear_acceleration.x = acc_calibrated[0];
    imu_msg.linear_acceleration.y = acc_calibrated[1];
    imu_msg.linear_acceleration.z = acc_calibrated[2];

    calibrated_pub.publish(imu_msg);

    // ---- Raw message (same header, uncalibrated payload) ----
    imu_msg.angular_velocity.x = angular_radians[0];
    imu_msg.angular_velocity.y = angular_radians[1];
    imu_msg.angular_velocity.z = angular_radians[2];
    imu_msg.linear_acceleration.x = acc_inverse[0];
    imu_msg.linear_acceleration.y = acc_inverse[1];
    imu_msg.linear_acceleration.z = acc_inverse[2];

    raw_pub.publish(imu_msg);
}

// ---------------------------------------------------------------------------
// Phidget spatial data callback
// (single handler used for all three IMUs; serial number used to dispatch)
// ---------------------------------------------------------------------------
void CCONV onSpatial0_SpatialData(PhidgetSpatialHandle ch, void *ctx,
                                  const double acceleration[3],  // acceleration is in negative g (1g = 9.81m/s^2)
                                  const double angularRate[3],   // angularRate is in deg/s
                                  const double magneticField[3], // magneticField is not used
                                  double timestamp)              // timestamp is in ms
{
    double now_in_s = ros::Time::now().toSec();
    int serialNr;
    int spatialNr;

    /* We invert the measured acceleration to get correct gravity direction */
    double accelerationInverse[3] = {
        -acceleration[0],
        -acceleration[1],
        -acceleration[2]};

    /* Angular rate should not be in deg/s but rad/s for downstream calculations */
    double angularRateRadians[3] = {
        angularRate[0] * M_PI_BY_180,
        angularRate[1] * M_PI_BY_180,
        angularRate[2] * M_PI_BY_180};

    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &serialNr);

    if (serialNr == SERIAL_0)
    {
        spatialNr = 0;
        if (!initialized0)
        {
            initialized0 = true;
            n_imus++;
            ROS_INFO_STREAM("Receiving data from IMU " << spatialNr
                                                       << " (serial " << serialNr << ") — OK");
        }
        process_phidget_to_calibrated_ros_msg(
            serialNr, spatialNr,
            accelerationInverse, angularRateRadians,
            sync0, now_in_s, timestamp, lastTime_serial0_ms,
            gyr0_misalignment, gyr0_scale, gyr0_bias,
            acc0_misalignment, acc0_scale, acc0_bias,
            acc0_calibrated, gyr0_calibrated,
            "imu0", seq0,
            imu0_msg, imu0_raw_pub, imu0_calib_pub);
    }
    else if (serialNr == SERIAL_1)
    {
        spatialNr = 1;
        if (!initialized1)
        {
            initialized1 = true;
            n_imus++;
            ROS_INFO_STREAM("Receiving data from IMU " << spatialNr
                                                       << " (serial " << serialNr << ") — OK");
        }
        process_phidget_to_calibrated_ros_msg(
            serialNr, spatialNr,
            accelerationInverse, angularRateRadians,
            sync1, now_in_s, timestamp, lastTime_serial1_ms,
            gyr1_misalignment, gyr1_scale, gyr1_bias,
            acc1_misalignment, acc1_scale, acc1_bias,
            acc1_calibrated, gyr1_calibrated,
            "imu1", seq1,
            imu1_msg, imu1_raw_pub, imu1_calib_pub);
    }
    else if (serialNr == SERIAL_2)
    {
        spatialNr = 2;
        if (!initialized2)
        {
            initialized2 = true;
            n_imus++;
            ROS_INFO_STREAM("Receiving data from IMU " << spatialNr
                                                       << " (serial " << serialNr << ") — OK");
        }
        process_phidget_to_calibrated_ros_msg(
            serialNr, spatialNr,
            accelerationInverse, angularRateRadians,
            sync2, now_in_s, timestamp, lastTime_serial2_ms,
            gyr2_misalignment, gyr2_scale, gyr2_bias,
            acc2_misalignment, acc2_scale, acc2_bias,
            acc2_calibrated, gyr2_calibrated,
            "imu2", seq2,
            imu2_msg, imu2_raw_pub, imu2_calib_pub);
    }
    else
    {
        ROS_WARN("Attention! IMU with serial %d was not declared in config.launch!", serialNr);
    }
}

// ---------------------------------------------------------------------------
// ROS parameter loading + YAML calibration
// ---------------------------------------------------------------------------
int argumentHandler(ros::NodeHandle &nh)
{
    // Rates
    int temp;
    nh.param<int>("jasper_pub_rate", temp, DATA_RATE_DEFAULT);
    if (temp > 0 && temp < 501)
    {
        data_rate = temp;
        data_intervall = 1.0f / temp;
        ROS_INFO("Publish rate set to %d Hz", data_rate);
    }
    else
    {
        ROS_WARN("jasper_pub_rate must be 1–500 Hz. Defaulting to %d Hz", DATA_RATE_DEFAULT);
    }
    nh.param<int>("jasper_imu_rate", imu_data_rate, IMU_DATA_RATE_DEFAULT);
    imu_data_rate = imu_data_rate > 250 ? 250 : imu_data_rate;

    // Debug flag
    nh.param<bool>("jasper_debug", debugMode, false);

    // Serial numbers
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
    ROS_WARN_STREAM_COND(use_serial0, "Using IMU serial " << SERIAL_0);
    ROS_WARN_STREAM_COND(use_serial1, "Using IMU serial " << SERIAL_1);
    ROS_WARN_STREAM_COND(use_serial2, "Using IMU serial " << SERIAL_2);

    initialized0 = false;
    initialized1 = false;
    initialized2 = false;
    n_imus = 0;

    /*
        INTRINSIC PARAMETERS

    The intrinsic paramters have been calibrated using
    https://github.com/fallow24/ros_imu_calib
    Parameters include misalignment between axes, scaling factor, and bias.
    */
    std::string acc_calib0, acc_calib1, acc_calib2;
    std::string gyr_calib0, gyr_calib1, gyr_calib2;
    nh.param<std::string>("jasper_acc_calib0", acc_calib0, "");
    nh.param<std::string>("jasper_acc_calib1", acc_calib1, "");
    nh.param<std::string>("jasper_acc_calib2", acc_calib2, "");
    nh.param<std::string>("jasper_gyr_calib0", gyr_calib0, "");
    nh.param<std::string>("jasper_gyr_calib1", gyr_calib1, "");
    nh.param<std::string>("jasper_gyr_calib2", gyr_calib2, "");

    YAML::Node acc_config0 = YAML::LoadFile(acc_calib0);
    YAML::Node acc_config1 = YAML::LoadFile(acc_calib1);
    YAML::Node acc_config2 = YAML::LoadFile(acc_calib2);
    YAML::Node gyr_config0 = YAML::LoadFile(gyr_calib0);
    YAML::Node gyr_config1 = YAML::LoadFile(gyr_calib1);
    YAML::Node gyr_config2 = YAML::LoadFile(gyr_calib2);

    acc0_misalignment = acc_config0["misalignment"].as<std::vector<double>>();
    acc0_scale = acc_config0["scale"].as<std::vector<double>>();
    acc0_bias = acc_config0["bias"].as<std::vector<double>>();
    acc1_misalignment = acc_config1["misalignment"].as<std::vector<double>>();
    acc1_scale = acc_config1["scale"].as<std::vector<double>>();
    acc1_bias = acc_config1["bias"].as<std::vector<double>>();
    acc2_misalignment = acc_config2["misalignment"].as<std::vector<double>>();
    acc2_scale = acc_config2["scale"].as<std::vector<double>>();
    acc2_bias = acc_config2["bias"].as<std::vector<double>>();
    gyr0_misalignment = gyr_config0["misalignment"].as<std::vector<double>>();
    gyr0_scale = gyr_config0["scale"].as<std::vector<double>>();
    gyr0_bias = gyr_config0["bias"].as<std::vector<double>>();
    gyr1_misalignment = gyr_config1["misalignment"].as<std::vector<double>>();
    gyr1_scale = gyr_config1["scale"].as<std::vector<double>>();
    gyr1_bias = gyr_config1["bias"].as<std::vector<double>>();
    gyr2_misalignment = gyr_config2["misalignment"].as<std::vector<double>>();
    gyr2_scale = gyr_config2["scale"].as<std::vector<double>>();
    gyr2_bias = gyr_config2["bias"].as<std::vector<double>>();

    return 0;
}

void CCONV detachHandler(PhidgetHandle ch, void *ctx)
{
    int serialNr;
    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &serialNr);
    ROS_ERROR("IMU with serial %d detached!", serialNr);
}

// ---------------------------------------------------------------------------
// Phidget lifecycle (create → address → open → zero gyro → assign cb)
// ---------------------------------------------------------------------------
int CCONV init()
{
    ROS_INFO("Initialising Phidget IMUs…");
    Phidget_resetLibrary();

    PhidgetSpatialHandle spatial0, spatial1, spatial2;

    // Create your Phidget channels
    if (use_serial0)
        PhidgetSpatial_create(&spatial0);
    if (use_serial1)
        PhidgetSpatial_create(&spatial1);
    if (use_serial2)
        PhidgetSpatial_create(&spatial2);

    // Set addressing parameters to specify which channel to open (if any)
    if (use_serial0)
        Phidget_setDeviceSerialNumber((PhidgetHandle)spatial0, SERIAL_0);
    if (use_serial1)
        Phidget_setDeviceSerialNumber((PhidgetHandle)spatial1, SERIAL_1);
    if (use_serial2)
        Phidget_setDeviceSerialNumber((PhidgetHandle)spatial2, SERIAL_2);

    // set the handler which handels detaching evcents
    if (use_serial0)
        Phidget_setOnDetachHandler((PhidgetHandle)spatial0, detachHandler, NULL);
    if (use_serial1)
        Phidget_setOnDetachHandler((PhidgetHandle)spatial1, detachHandler, NULL);
    if (use_serial2)
        Phidget_setOnDetachHandler((PhidgetHandle)spatial2, detachHandler, NULL);

    ROS_INFO("Now Attaching the IMUS! Giving it maximum 3 seconds!");
    // Open your Phidgets and wait for attachment
    ros::Time begin = ros::Time::now();
    if (use_serial0)
        Phidget_openWaitForAttachment((PhidgetHandle)spatial0, 1000);
    if (use_serial1)
        Phidget_openWaitForAttachment((PhidgetHandle)spatial1, 1000);
    if (use_serial2)
        Phidget_openWaitForAttachment((PhidgetHandle)spatial2, 1000);
    if (ros::Time::now().toSec() - begin.toSec() >= 3.0)
    {
        ROS_WARN("Phidget IMU attachment timeout — data may be compromised.");
    }
    else
    {
        ROS_INFO("Phidget IMU attachment successful.");
    }

    // Set the data rate (set by user or IMU_DEFAULT_DATA_RATE
    int imu_data_intervall = std::round(1000.0 / imu_data_rate);
    if (use_serial0)
        PhidgetSpatial_setDataInterval(spatial0, imu_data_intervall);
    if (use_serial1)
        PhidgetSpatial_setDataInterval(spatial1, imu_data_intervall);
    if (use_serial2)
        PhidgetSpatial_setDataInterval(spatial2, imu_data_intervall);

    // Zero gyroscopes — robot must be stationary
    ROS_WARN("!!! CALIBRATING GYROSCOPES — DO NOT MOVE IMUs !!!");
    ros::Duration(0.5).sleep();
    if (use_serial0)
        PhidgetSpatial_zeroGyro(spatial0);
    if (use_serial1)
        PhidgetSpatial_zeroGyro(spatial1);
    if (use_serial2)
        PhidgetSpatial_zeroGyro(spatial2);
    if (use_serial0)
        PhidgetSpatial_zeroAlgorithm(spatial0);
    if (use_serial1)
        PhidgetSpatial_zeroAlgorithm(spatial1);
    if (use_serial2)
        PhidgetSpatial_zeroAlgorithm(spatial2);
    ROS_WARN("Gyroscope zeroing complete.");

    // Assign any event handlers you need before calling open so that no events are missed.
    if (use_serial0)
        PhidgetSpatial_setOnSpatialDataHandler(spatial0, onSpatial0_SpatialData, NULL);
    if (use_serial1)
        PhidgetSpatial_setOnSpatialDataHandler(spatial1, onSpatial0_SpatialData, NULL);
    if (use_serial2)
        PhidgetSpatial_setOnSpatialDataHandler(spatial2, onSpatial0_SpatialData, NULL);

    ROS_INFO("Phidget initialisation complete.");
    return 0;
}

void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imuHandling", ros::init_options::NoSigintHandler);
    ROS_INFO("Starting Node imuJasper");
    if (argc > 1)
    {
        ROS_INFO("Command-line args are not supported anymore.");
        ROS_INFO("Please use the ROS parameter server and have a look at the config.launch file!");
    }
    signal(SIGINT, mySigIntHandler);

    // Arguments as nodehandle parameters (from ROS server)
    ros::NodeHandle nH;
    argumentHandler(nH);

    // Allocate calibration buffers
    acc0_calibrated = new double[3];
    gyr0_calibrated = new double[3];
    acc1_calibrated = new double[3];
    gyr1_calibrated = new double[3];
    acc2_calibrated = new double[3];
    gyr2_calibrated = new double[3];

    // Advertise topics
    imu0_raw_pub = nH.advertise<sensor_msgs::Imu>("/imu/0/raw", 1000);
    imu1_raw_pub = nH.advertise<sensor_msgs::Imu>("/imu/1/raw", 1000);
    imu2_raw_pub = nH.advertise<sensor_msgs::Imu>("/imu/2/raw", 1000);
    imu0_calib_pub = nH.advertise<sensor_msgs::Imu>("/imu/0/calib", 1000);
    imu1_calib_pub = nH.advertise<sensor_msgs::Imu>("/imu/1/calib", 1000);
    imu2_calib_pub = nH.advertise<sensor_msgs::Imu>("/imu/2/calib", 1000);

    init();

    // spin — all work happens in the Phidget callbacks
    ros::Rate loop_rate(data_rate);
    while (ros::ok() && !g_request_shutdown)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN("imu_jasper shutdown — goodbye!");
    Phidget_resetLibrary();
    ros::shutdown();
    return 0;
}
