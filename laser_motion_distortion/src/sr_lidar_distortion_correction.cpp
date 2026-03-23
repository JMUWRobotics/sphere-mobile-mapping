// Lidar Distortion Correction for spherical robots

#include "sr_lidar_distortion_correction.h"

using ::ros::NodeHandle;
using namespace std;


SRLidarDistortionCorrection::SRLidarDistortionCorrection()
        : node_initialized_(false),
          system_ready_(false),
          spin_rate_(1000.0)
{
}

// Spins the callback queue
void SRLidarDistortionCorrection::run()
{
    if (!node_initialized_)
    {
        ROS_FATAL("SRLidarDistortionCorrection is not initialized. Shutdown.");
        return;
    }
    ros::spin();
}

// Initialize node
bool SRLidarDistortionCorrection::init() {
  
  // Read from rosparam server
  if ( readConfig())
  {
    node_initialized_ = true;
  }
  // create publishers
  pcd_undistorted_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("points_out", 1);

  // create subscribers
  pcd_in_sub_ = node_handle_.subscribe("points_in", 10,
      &SRLidarDistortionCorrection::pcdCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  imu_in_sub_ = node_handle_.subscribe<state_estimator_msgs::Estimator>("imu_in", 1000,
      &SRLidarDistortionCorrection::vehicleTwistCallback,
      this, ros::TransportHints().tcpNoDelay(true));

  // Get the imu -> lidar transformation
  static tf::TransformListener listener_tf;
  try {
    // FROM Imu TO Pandar frame: Premultiply imu2lidar_tf 
    listener_tf.waitForTransform("/pandar_frame", "/imu_frame", ros::Time(0), ros::Duration(10.0));
    listener_tf.lookupTransform("/pandar_frame", "/imu_frame", ros::Time(0), imu2lidar_tf);
  } catch (tf::TransformException &e) {
    ROS_ERROR("%s", e.what());
    tf::Transform no_tf(tf::Quaternion(0,0,0,1));
    imu2lidar_tf.setData(no_tf);
  }

  return node_initialized_;
}

bool SRLidarDistortionCorrection::readConfig()
{
    ros::NodeHandle priv_nh("~");

    // Config arguments
    priv_nh.param<int>("radius_filter", radius_filter, -1);
    if (radius_filter != -1) 
      radius_sqr_m = 0.0001*radius_filter*radius_filter;
    else
      radius_sqr_m = 0;
    return true;
}


void SRLidarDistortionCorrection::process(const ros::TimerEvent &)
{
  ros::Time current_time = ros::Time::now();
}

// Callback function for ROS point cloud msgs. 
// Gets the vehicle twist during the LiDAR measurement from the IMU buffer
// and applies angular motion distortion correction by integrating angular velocity.
void SRLidarDistortionCorrection::pcdCallback(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  // Convert ROS to PCL (use custom PointType to access timestamp fields of Hesai Lidar)
  pcl::PointCloud<PointType>::Ptr pcd_in (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*pc, *pcd_in);
  // Point towards private class attribute
  pcd_in_ = pcd_in;

  double scan_start = pc->header.stamp.toSec();

  // Prune old IMU data, keeping one sample before scan_start for interpolation
  while (imu_buffer_.size() > 1 && imu_buffer_[1].timestamp <= scan_start) {
    imu_buffer_.pop_front();
  }

  if (imu_buffer_.size() < 2) {
    ROS_WARN_THROTTLE(1.0, "Not enough IMU data for distortion correction");
    pcd_undistorted_pub_.publish(*pc);
    return;
  }

  // --- Precompute cumulative rotation quaternions via omega matrix integration ---
  // q_dot = 1/2 * Omega(w) * q  =>  exact step: q_new = quat(axis, theta) * q_old
  struct CumQuat { double t; Eigen::Quaterniond q; };
  std::vector<CumQuat> cum;
  cum.reserve(imu_buffer_.size() + 1);
  cum.push_back({scan_start, Eigen::Quaterniond::Identity()});

  // Interpolate angular velocity at scan_start between the surrounding IMU samples
  Eigen::Vector3d w_prev;
  if (imu_buffer_[0].timestamp < scan_start) {
    double alpha = (scan_start - imu_buffer_[0].timestamp) /
                   (imu_buffer_[1].timestamp - imu_buffer_[0].timestamp);
    w_prev = (1.0 - alpha) * Eigen::Vector3d(imu_buffer_[0].wx, imu_buffer_[0].wy, imu_buffer_[0].wz)
           + alpha * Eigen::Vector3d(imu_buffer_[1].wx, imu_buffer_[1].wy, imu_buffer_[1].wz);
  } else {
    w_prev = Eigen::Vector3d(imu_buffer_[0].wx, imu_buffer_[0].wy, imu_buffer_[0].wz);
  }

  Eigen::Quaterniond q_cum = Eigen::Quaterniond::Identity();
  double prev_t = scan_start;

  for (size_t i = 0; i < imu_buffer_.size(); ++i) {
    if (imu_buffer_[i].timestamp <= scan_start) continue;
    double dt = imu_buffer_[i].timestamp - prev_t;
    Eigen::Vector3d w_cur(imu_buffer_[i].wx, imu_buffer_[i].wy, imu_buffer_[i].wz);
    // Average angular velocity over the interval
    Eigen::Vector3d w_avg = 0.5 * (w_prev + w_cur);
    double theta = w_avg.norm() * dt;
    if (theta > 1e-12) {
      q_cum = Eigen::Quaterniond(Eigen::AngleAxisd(theta, w_avg.normalized())) * q_cum;
    }
    q_cum.normalize();
    cum.push_back({imu_buffer_[i].timestamp, q_cum});
    w_prev = w_cur;
    prev_t = imu_buffer_[i].timestamp;
  }

  // --- Undistort each point ---
  pcl::PointCloud<pcl::PointXYZI> pcl_out;
  pcl_out.reserve(pcd_in->size());
  double scan_x, scan_y, scan_z;

  for (unsigned int i = 0; i < pcd_in_->size(); ++i) {
    double t_pt = pcd_in_->points[i].timestamp;

    // read point coordinates
    scan_x = pcd_in_->points[i].x;
    scan_y = pcd_in_->points[i].y;
    scan_z = pcd_in_->points[i].z;

    // check for validity
    if(std::isnan(scan_x) || std::isnan(scan_y) || std::isnan(scan_z)){continue;}
    // optional distance filter
    if (radius_filter != -1 && (scan_x*scan_x+scan_y*scan_y+scan_z*scan_z < radius_sqr_m)){continue;}

    // Look up cumulative rotation quaternion at this point's timestamp
    Eigen::Quaterniond q_pt = Eigen::Quaterniond::Identity();
    if (t_pt <= cum.front().t) {
      // Before scan start: no correction
    } else if (t_pt >= cum.back().t) {
      // Beyond last IMU sample: extrapolate using last angular velocity
      double dt_extra = t_pt - cum.back().t;
      double theta = w_prev.norm() * dt_extra;
      if (theta > 1e-12) {
        q_pt = Eigen::Quaterniond(Eigen::AngleAxisd(theta, w_prev.normalized())) * cum.back().q;
      } else {
        q_pt = cum.back().q;
      }
    } else {
      // Binary search for the interval containing t_pt
      size_t lo = 0, hi = cum.size() - 1;
      while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if (cum[mid].t <= t_pt) lo = mid;
        else hi = mid;
      }
      double alpha = (t_pt - cum[lo].t) / (cum[hi].t - cum[lo].t);
      q_pt = cum[lo].q.slerp(alpha, cum[hi].q);
    }

    // Apply quaternion rotation to point
    Eigen::Vector3d p_out = q_pt * Eigen::Vector3d(scan_x, scan_y, scan_z);

    pcl::PointXYZI pt;
    pt.x = p_out.x();
    pt.y = p_out.y();
    pt.z = p_out.z();

    // check for validity
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)){continue;}
    pt.intensity = pcd_in_->points[i].intensity;
    pcl_out.push_back(pt);
  }

  // Construct new ROS msg
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_out));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan_ptr, msg);
  msg.header = pc->header;
  
  // publish
  pcd_undistorted_pub_.publish(msg);
}

void SRLidarDistortionCorrection::vehicleTwistCallback(const state_estimator_msgs::EstimatorConstPtr &imu)
{
  // Transform angular velocity from IMU frame to LiDAR frame and buffer
  tf::Vector3 w_imu(imu->imu.angular_velocity.x,
                    imu->imu.angular_velocity.y,
                    imu->imu.angular_velocity.z);
  tf::Vector3 w_lidar = imu2lidar_tf * w_imu;

  ImuSample sample;
  sample.timestamp = imu->header.stamp.toSec();
  sample.wx = w_lidar.x();
  sample.wy = w_lidar.y();
  sample.wz = w_lidar.z();
  imu_buffer_.push_back(sample);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "SR_lidar_distortion_correction_node");

  SRLidarDistortionCorrection sr_lidar_distortion_correction_node;
  if (sr_lidar_distortion_correction_node.init()) {
    sr_lidar_distortion_correction_node.run(); 
  }
  else {
    ROS_FATAL_STREAM("SR_lidar_distortion_correction_node initialization failed. Shutdown.");
  }
  return 0;
}
