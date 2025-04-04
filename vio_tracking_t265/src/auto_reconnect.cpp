// System
#include <cmath>
#include <thread>
#include <mutex>
#include <memory>
#include <omp.h>
#include <condition_variable>

// Librealsense2
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/h/rs_types.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// Custom msg
#include <state_estimator_msgs/Estimator.h>

const char *pose_topic = "/camera/pose";
const char *cam_imu_topic = "/camera/imu";
const char *cam_pose_imu_topic = "/camera/poseAndImu";
const char *cam_images_topic_1 = "/camera/images/1";
const char *cam_images_topic_2 = "/camera/images/2";
const char *camera_frame = "camera_frame";

// Setup ros publishers
std::shared_ptr<ros::Publisher> cam_pose;
std::shared_ptr<ros::Publisher> cam_imu;
std::shared_ptr<ros::Publisher> cam_pose_imu;
std::shared_ptr<image_transport::Publisher> cam_imgs_1;
std::shared_ptr<image_transport::Publisher> cam_imgs_2;

// ROS msg objects
sensor_msgs::Imu imu_msg;
geometry_msgs::PoseStamped output_msg;
state_estimator_msgs::Estimator published_msg;
sensor_msgs::Image *image_msg1;
sensor_msgs::Image *image_msg2;

// Flag to indicate if we should reset
bool reset_rs2_pipeline = false;

// Thread objects needed for video processing (two threads)
std::condition_variable video1_condition;
std::condition_variable video2_condition;
std::shared_ptr<rs2::video_frame> camera1_data;
std::shared_ptr<rs2::video_frame> camera2_data;
std::mutex camera_data_mtx;
std::mutex video_mtx1;
std::mutex video_mtx2; 

void rosHandleRS2VideoFrame1()
{
	// Thread should always run (except ROS shuts down)
	ros::Rate frequency(100);
	while (ros::ok()) {
		
		// But we should block the thread until there is some video data 
		std::unique_lock<std::mutex> lock_video(video_mtx1);
		video1_condition.wait(lock_video, [&]{
			// Check if the condition is actually met upon unblocking
			return camera1_data.get() != nullptr;
		});

		// If video frame arrives do the following:
		// Header and metadata
		image_msg1->header.frame_id = camera_frame;
		image_msg1->header.stamp = ros::Time::now();
		image_msg1->height = camera1_data->get_height();
		image_msg1->width = camera1_data->get_width();
		int stepSize = camera1_data->get_stride_in_bytes();
		image_msg1->step = stepSize;
		image_msg1->encoding = "mono8";
		int dataSize = image_msg1->height * stepSize;

		// copy data
		std::vector<uint8_t> *data_vctr_ptr = new std::vector<uint8_t>();
		image_msg1->data = *data_vctr_ptr;
		image_msg1->data.resize(dataSize);
		image_msg1->is_bigendian = false;
		if (&image_msg1->data[0]) {
			memcpy(&image_msg1->data[0], camera1_data->get_data(), dataSize);
			cam_imgs_1->publish(std::move(*image_msg1));
		}
		delete data_vctr_ptr;
		frequency.sleep();
	}
	return;
}

void rosHandleRS2VideoFrame2()
{
	// Thread should always run (except ROS shuts down)
	ros::Rate frequency(100);
	while (ros::ok()) {
		
		// But we should block the thread until there is some video data 
		std::unique_lock<std::mutex> lock_video(video_mtx2);
		video2_condition.wait(lock_video, [&]{
			// Check if the condition is actually met upon unblocking
			return camera2_data.get() != nullptr;
		});

		// If video frame arrives do the following:
		// Header and metadata
		image_msg2->header.frame_id = camera_frame;
		image_msg2->header.stamp = ros::Time::now();
		image_msg2->height = camera2_data->get_height();
		image_msg2->width = camera2_data->get_width();
		int stepSize = camera2_data->get_stride_in_bytes();
		image_msg2->step = stepSize;
		image_msg2->encoding = "mono8";
		int dataSize = image_msg2->height * stepSize;

		// copy data
		std::vector<uint8_t> *data_vctr_ptr = new std::vector<uint8_t>();
		image_msg2->data = *data_vctr_ptr;
		image_msg2->data.resize(dataSize);
		image_msg2->is_bigendian = false;
		if (&image_msg2->data[0]) {
			memcpy(&image_msg2->data[0], camera2_data->get_data(), dataSize);
			cam_imgs_2->publish(std::move(*image_msg2));
		}
		delete data_vctr_ptr;
		frequency.sleep();
	}
	return;
}

/**
 * Publishes a realsense2 frame to a ROS topic
 */
int rosPublishRS2Frame(const rs2::frame &frame)
{
	// 6 DoF pose data
	if (auto pose_frame = frame.as<rs2::pose_frame>())
	{
		auto pose_data = frame.as<rs2::pose_frame>().get_pose_data();

		// Sanity check for NaN
		if (std::isnan(pose_data.translation.x) ||
				std::isnan(pose_data.translation.y) ||
				std::isnan(pose_data.translation.z) ||
				std::isnan(pose_data.rotation.x) ||
				std::isnan(pose_data.rotation.y) ||
				std::isnan(pose_data.rotation.z) ||
				std::isnan(pose_data.rotation.w))
		{
			ROS_WARN("Pose data contains NaN values.");
			reset_rs2_pipeline = true;
			return 0;
		}

		/* Convert everything into ROS geometry_msgs::PoseStamped 
		The official ROS wrapper does the same according to:
		https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_camera/src/base_realsense_node.cpp#L1556*/
		output_msg.header.frame_id = "map";
		output_msg.header.stamp = ros::Time::now();
		output_msg.pose.position.x = -pose_data.translation.z;
		output_msg.pose.position.y = -pose_data.translation.x;
		output_msg.pose.position.z = pose_data.translation.y;
		output_msg.pose.orientation.x = -pose_data.rotation.z;
		output_msg.pose.orientation.y = -pose_data.rotation.x;
		output_msg.pose.orientation.z = pose_data.rotation.y;
		output_msg.pose.orientation.w = pose_data.rotation.w;
		cam_pose->publish(output_msg);

		// Save camera pose confidence (0 = Failed, 1 = Low, 2 = Medium, 3 = High confidence)
		// Tracker confidence is published later together with motion data		
		published_msg.tracker_confidence = pose_data.tracker_confidence; 

		// Also fill IMU msg with redundant data
		imu_msg.orientation = output_msg.pose.orientation;
		return 1;
	}

	// 3 DoF gyroscope data
	else if (auto motion_frame = frame.as<rs2::motion_frame>())
	{
		auto imu_data = frame.as<rs2::motion_frame>().get_motion_data();
	
		// Sanity check for NaN
		if (std::isnan(imu_data.x) ||
			std::isnan(imu_data.y) ||
			std::isnan(imu_data.z))
		{
			ROS_WARN("Motion data contains NaN values.");
			reset_rs2_pipeline = true;
			return 0;
		}

		// Convert everything into ROS sensor_msgs::Imu
		imu_msg.header.frame_id = "map";
		imu_msg.header.stamp = ros::Time::now();

		/* This is how the POSE conversion to ROS would look like: */
		//	imu_msg.angular_velocity.x = -imu_data.z;
		//	imu_msg.angular_velocity.y = -imu_data.x;
		//	imu_msg.angular_velocity.z = imu_data.y;
		/* However, the IMU has inverted X and Z axis according to
		https://github.com/IntelRealSense/realsense-ros/issues/912 */
		imu_msg.angular_velocity.x = imu_data.z;
		imu_msg.angular_velocity.y = -imu_data.x;
		imu_msg.angular_velocity.z = -imu_data.y;

		// Wrap camera pose and cameras imu data into custom message type
		published_msg.pose = output_msg;
		published_msg.imu = imu_msg;
		published_msg.header = output_msg.header;

		// Publish on ROS topic
		cam_imu->publish(imu_msg);
		cam_pose_imu->publish(published_msg);
		return 1;
	}

	else if (auto image_frame = frame.as<rs2::video_frame>())
	{	
		auto camera_data = std::make_shared<rs2::video_frame>(frame.as<rs2::video_frame>());
		// Handle First Camera
		if (camera_data->get_profile().stream_index() == 1) {
			camera1_data = camera_data;
			video1_condition.notify_one();
		}
		// Handle Second Camera
		else {
			camera2_data = camera_data;	
			video2_condition.notify_one();
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	// Mutex lock for frame pipeline
	std::mutex mutex;
	rs2::syncer syncer;
	
	/**
	 * Callback function for librealsense frames.
	 * This function will recognize if the frame contains motion, pose, or image data.
	 * Then it will publish the frame as a ros msgs accordingly.
	 */
	auto rs2FramesCallback = [&](const rs2::frame &frame)
	{
		// Lock the mutex as we access common memory from multiple threads
		std::lock_guard<std::mutex> lock(mutex);

		// All synchronized stream will arrive in a single frameset
		if (rs2::frameset fs = frame.as<rs2::frameset>())
		{
			for (const rs2::frame &f : fs)
			{
				syncer(f);
			}
		}
		// Stream that bypass synchronization (such as IMU) will produce single frames
		else
		{
			syncer(frame);
		}
	};

	// Setting up ROS Nodehandle
	ros::init(argc, argv, "custom_t265_node");
	ROS_INFO("Custom T265 Intel node started. Publishing %s", pose_topic);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	cam_pose = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000));
	cam_imu = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::Imu>(cam_imu_topic, 1000));
	cam_pose_imu = std::make_shared<ros::Publisher>(nh.advertise<state_estimator_msgs::Estimator>(cam_pose_imu_topic, 1000));
	cam_imgs_1 = std::make_shared<image_transport::Publisher>(it.advertise(cam_images_topic_1, 1));
	cam_imgs_2 = std::make_shared<image_transport::Publisher>(it.advertise(cam_images_topic_2, 1));
	
	// Init Realsense camera
	// Setup Librealsense2 config
	rs2::log_to_console(RS2_LOG_SEVERITY_INFO); // verbose, can make this _INFO for less or _DEBUG for more
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 1, 848, 800, RS2_FORMAT_Y8);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 2, 848, 800, RS2_FORMAT_Y8); // we only need left image

	// Create librealsense2 pipeline object which gets the data
	ROS_INFO("Setting up Pipeline");
	auto pipe = std::make_shared<rs2::pipeline>();
	// we have to start the pipeline first
	pipe->start(cfg, rs2FramesCallback);
	// then we can get the active profile which includes the sensor
	rs2::pipeline_profile profiles = pipe->get_active_profile();
	auto sensor = profiles.get_device().first<rs2::pose_sensor>();
	// stop the pipeline such that we can alter the options of the sensor
	pipe->stop();
	// set options (disable pose jumps) and restart pipeline with new options
	sensor.set_option(rs2_option::RS2_OPTION_ENABLE_POSE_JUMPING, 0);
	sensor.set_option(rs2_option::RS2_OPTION_ENABLE_RELOCALIZATION, 0);
	pipe->start(cfg, rs2FramesCallback);

	// allocate image memory 
	image_msg1 = new sensor_msgs::Image();
	image_msg2 = new sensor_msgs::Image();
	ros::Rate rate(1000); // Hz 

	// Setup the video threads
	std::thread video_handler_thread1(rosHandleRS2VideoFrame1);
	video_handler_thread1.detach();
	std::thread video_handler_thread2(rosHandleRS2VideoFrame2);
	video_handler_thread2.detach();

	ROS_INFO("Video threads started and detached. Now entering main loop");
	// Main thread loop, gets frames data from camera directly, checks for reset, publishes to ROS
	while (ros::ok())
	{
		if (reset_rs2_pipeline)
		{
			ROS_WARN("Resetting pipeline now.");
			reset_rs2_pipeline = false;
			pipe->stop();
			pipe = std::make_shared<rs2::pipeline>();
			pipe->start(cfg, rs2FramesCallback);
			ROS_WARN("Success! T265 pipe reset.");
		}
		auto frameset = syncer.wait_for_frames(5000);

		for (size_t i = 0; i < frameset.size(); ++i)
		{
			// This function publishes motion / pose data directly, and feeds video data to the other threads
			rosPublishRS2Frame(frameset[i]);
		}
		rate.sleep();
	}

	// Once ROS is no longer ok, force terminate all threads spawned by this process.
	std::terminate();
	return 0;
} // end main
