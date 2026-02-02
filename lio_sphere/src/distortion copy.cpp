#include "lio_node.hpp"
bool LIONode::distort(pcl::PointCloud<custom_type::PointXYZITR>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_out, Sophus::SE3d& initial_pose){
    
    //get corresponding IMU measurements
    double end_time = pc_in->points[pc_in->points.size()-1].timestamp;
    if (imu_buffer_.empty() || imu_buffer_.front().stamp < end_time){
        std::unique_lock<decltype(mtx_imu_)> lock(mtx_imu_);
        cv_imu_stamp_.wait(lock, [this, &end_time]{ return this->imu_buffer_.front().stamp >= end_time; });
    }

    auto lidar_it = pc_in->points.begin();
    auto lidar_start = *lidar_it;
    auto imu_it = imu_buffer_.begin();
    auto last_imu_it = imu_it; //points to the last IMU measurement during the LiDAR sweep interval
    imu_it++;

    while (imu_it != imu_buffer_.end() && imu_it->stamp >= end_time) {
        last_imu_it = imu_it; //iterate till last_imu_it points to last measurement stamped after LiDAR sweep end (temporally)
        imu_it++;
    }

    while (imu_it != imu_buffer_.end() && imu_it->stamp >= lidar_start.timestamp) {
        imu_it++; //iterate till imu_it points to the first measurement stamped before the LiDAR sweep start (temporally)
    }

    if (imu_it == this->imu_buffer_.end()){
        //buffer is to small or not enough IMU measurements collected
        ROS_WARN("Buffer is too small or no IMU data for LiDAR sweep begin available.");
        return false;
    }
    imu_it++;

    //create reverse_iterators to be able to iterate forward in time, the reverse iterators point to the measurements
    //that are one timestamp newer than the ones the normal iterators point to
    auto end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it); //points to the second measurement after LiDAR sweep end (temporally)
    auto begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it); //points to the last measurement before LiDAR sweep start (temporally)
    
    //set initial orientation
    tf2::Quaternion q(0,0,0,1);

    ImuMeas& imu_0 = *(begin_imu_it);
    ImuMeas& imu_1 = *(begin_imu_it+1); //first IMU measurement after LiDAR sweep start
    
    
    //  process points before first IMU measurement in LiDAR sweep
    //double dt = imu_1.stamp - imu_0.stamp;
    //  angular acceleration alpha
    //tf2::Vector3 alpha = (imu_1.ang_vel - imu_0.ang_vel)/dt;
    //  angular velocity at the beginning of the LiDAR sweep
    tf2::Vector3 omega_0 = imu_0.ang_vel.lerp(imu_1.ang_vel, (lidar_start.timestamp-imu_0.stamp));
    //  mean angular velocity between the LiDAR sweep start and the current LiDAR point
    tf2::Vector3 omega;
    Eigen::Vector3d omega_eigen;


    tf2::Vector3 trans_corr(0,0,0);
    tf2::Vector3 rot_prev;
    pcl::PointXYZ point_corr;
    tf2::Vector3 rot;
    tf2::Vector3 t_omega;
    tf2::Vector3 t_rot;
    tf2::Vector3 temp_trans_corr;
    double dt;
    double dt_i;
    //Sophus::SE3d relative_pose = Sophus::SE3d(Sophus::SE3d::QuaternionType(1,0,0,0), Sophus::SE3d::Point(0,0,0));
    Eigen::Quaterniond relative_orientation(1,0,0,0);
    Eigen::Vector4d orientation(1,0,0,0);
    Eigen::Quaterniond quat_orientation(1,0,0,0);
    Eigen::Matrix4d omega_operator;
    //correction for measurements between imu_0->imu_1
    lidar_it++;
    double dt_imu = imu_1.stamp - lidar_start.timestamp;
    while (imu_1.stamp > lidar_it->timestamp && lidar_it != pc_in->points.end()){
        dt_i = lidar_it->timestamp - (lidar_it-1)->timestamp;
        dt = (lidar_it-1)->timestamp +dt_i/2 - lidar_start.timestamp; //time between LiDAR sweep start and current LiDAR point
        omega = omega_0.lerp(imu_1.ang_vel, dt/dt_imu);//mean angular velocity between lidar_start and lidar point
        rot = omega * dt_i;
        //rotation correction
        //using the integration method described here: https://ahrs.readthedocs.io/en/latest/filters/angular.html
        omega_eigen=Eigen::Vector3d(omega.x(), omega.y(), omega.z());
        omega_operator.block<1,1>(0,0).setZero();
        omega_operator.block<1,3>(0,1) = -omega_eigen.transpose();
        omega_operator.block<3,1>(1,0) = omega_eigen;
        omega_operator.block<3,3>(1,1) = - Sophus::SO3d::hat(omega_eigen);
        orientation = (Eigen::Matrix4d::Identity() + 0.5 * omega_operator * dt_i)*orientation;
        quat_orientation = Eigen::Quaterniond(orientation[0], orientation[1], orientation[2], orientation[3]);

        //relative_pose *= Sophus::SE3d(eulerToQuat(rot.x(), rot.y(), rot.z()), Sophus::SE3d::Point(0,0,0));
        relative_orientation = eulerToQuat(rot.x(), rot.y(), rot.z()).inverse()*relative_orientation;
        Eigen::Quaterniond point= Eigen::Quaterniond(0, lidar_it->x, lidar_it->y, lidar_it->z);
        //Eigen::Quaterniond point_rot = relative_pose.unit_quaternion().inverse()*point*relative_pose.unit_quaternion();
        Eigen::Quaterniond point_rot = quat_orientation.inverse()*point*quat_orientation;
        //Eigen::Vector3d point= Eigen::Vector3d(lidar_it->x, lidar_it->y, lidar_it->z);
        //Eigen::Vector3d point_rot = relative_pose*point;
        point_corr.x = point_rot.x();
        point_corr.y =  point_rot.y();
        point_corr.z =  point_rot.z();
        
        //translation correction
        // if (lidar_it->timestamp > lidar_start.timestamp){
        //     t_omega = (lidar_it->timestamp-(lidar_it-1)->timestamp)*alpha*0.5;
        //     t_rot = t_omega * (lidar_it->timestamp-(lidar_it-1)->timestamp);
        //     temp_trans_corr;
        //     temp_trans_corr.setX((cos(rot.y())*cos(rot.z())) * trans_corr.x() + (sin(rot.x())*sin(rot.y())*cos(rot.z())-cos(rot.x())*sin(rot.z())) * trans_corr.y() + (cos(rot.x())*sin(rot.y())*cos(rot.z())+sin(rot.x())*sin(rot.z()))*trans_corr.z());
        //     temp_trans_corr.setY((cos(rot.y())*sin(rot.z())) * trans_corr.x() + (sin(rot.x())*sin(rot.y())*sin(rot.z())+cos(rot.x())*cos(rot.z())) * trans_corr.y() + (cos(rot.x())*sin(rot.y())*sin(rot.z())-sin(rot.x())*cos(rot.z()))*trans_corr.z());
        //     temp_trans_corr.setZ((-sin(rot.y())) * trans_corr.x() + (sin(rot.x())*cos(rot.y())) * trans_corr.y() + (cos(rot.x())*cos(rot.y())) * trans_corr.z());
        //     trans_corr = temp_trans_corr -lidar_offset.cross(t_rot*(2/M_PI));
        // }
        // point_corr.x += trans_corr.x();
        // point_corr.y += trans_corr.y();
        // point_corr.z += trans_corr.z();

        pc_out->push_back(point_corr);
        lidar_it++;
    }
    initial_pose = imu_0.pose;
    
    begin_imu_it++;
    auto imu_corr_it = begin_imu_it;

     for(; imu_corr_it != end_imu_it; imu_corr_it++){
        imu_0 = *imu_corr_it;
        imu_1 = *(imu_corr_it+1);
        omega_0 = imu_0.ang_vel;
        dt_imu = imu_1.stamp - imu_0.stamp;
        //dt = imu_1.stamp - imu_0.stamp;
        //alpha = (imu_1.ang_vel-imu_0.ang_vel)/dt;
        //tf::lerp() for interpolation
    
        while (imu_1.stamp > lidar_it->timestamp && lidar_it != pc_in->points.end()){
            dt_i = lidar_it->timestamp - (lidar_it-1)->timestamp;
            dt = (lidar_it-1)->timestamp+dt_i/2-imu_0.stamp;
            omega = imu_0.ang_vel.lerp(imu_1.ang_vel, dt/dt_imu); // mean angular velocity between previous imu_measurement and current lidar point
            rot = omega * dt_i;
            //rotation correction
            //using the integration method described here: https://ahrs.readthedocs.io/en/latest/filters/angular.html
            omega_eigen=Eigen::Vector3d(omega.x(), omega.y(), omega.z());
            omega_operator.block<1,1>(0,0).setZero();
            omega_operator.block<1,3>(0,1) = -omega_eigen.transpose();
            omega_operator.block<3,1>(1,0) = omega_eigen;
            omega_operator.block<3,3>(1,1) = - Sophus::SO3d::hat(omega_eigen);
            orientation = (Eigen::Matrix4d::Identity() + 0.5 * omega_operator * dt_i)*orientation;
            quat_orientation = Eigen::Quaterniond(orientation[0], orientation[1], orientation[2], orientation[3]);

            relative_orientation = eulerToQuat(rot.x(), rot.y(), rot.z()).inverse()*relative_orientation;
            Eigen::Quaterniond point= Eigen::Quaterniond(0, lidar_it->x, lidar_it->y, lidar_it->z);
            //Eigen::Quaterniond point_rot = relative_pose.unit_quaternion().inverse()*point*relative_pose.unit_quaternion();
            Eigen::Quaterniond point_rot = quat_orientation.inverse()*point*quat_orientation;     
            //Eigen::Vector3d point= Eigen::Vector3d(lidar_it->x, lidar_it->y, lidar_it->z);
            //Eigen::Vector3d point_rot = relative_pose*point;
            point_corr.x = point_rot.x();
            point_corr.y =  point_rot.y();
            point_corr.z =  point_rot.z();
            //translation correction
            // if (lidar_it->timestamp > lidar_start.timestamp){
            //     t_omega = (lidar_it->timestamp-(lidar_it-1)->timestamp)*alpha*0.5;
            //     t_rot = t_omega * (lidar_it->timestamp-(lidar_it-1)->timestamp);
            //     temp_trans_corr;
            //     temp_trans_corr.setX((cos(rot.y())*cos(rot.z())) * trans_corr.x() + (sin(rot.x())*sin(rot.y())*cos(rot.z())-cos(rot.x())*sin(rot.z())) * trans_corr.y() + (cos(rot.x())*sin(rot.y())*cos(rot.z())+sin(rot.x())*sin(rot.z()))*trans_corr.z());
            //     temp_trans_corr.setY((cos(rot.y())*sin(rot.z())) * trans_corr.x() + (sin(rot.x())*sin(rot.y())*sin(rot.z())+cos(rot.x())*cos(rot.z())) * trans_corr.y() + (cos(rot.x())*sin(rot.y())*sin(rot.z())-sin(rot.x())*cos(rot.z()))*trans_corr.z());
            //     temp_trans_corr.setZ((-sin(rot.y())) * trans_corr.x() + (sin(rot.x())*cos(rot.y())) * trans_corr.y() + (cos(rot.x())*cos(rot.y())) * trans_corr.z());
            //     trans_corr = temp_trans_corr -lidar_offset.cross(t_rot*(2/M_PI));
            //     tf2::Quaternion q;
            //     q.setRPY(t_rot.x(), t_rot.y(), t_rot.z());
            //     pose_pred = Sophus::SE3d(Sophus::SE3d::QuaternionType(q.w(), q.x(), q.y(), q.z()), Sophus::SE3d::Point(0,0,0));
                
            // }
            // point_corr.x += trans_corr.x();
            // point_corr.y += trans_corr.y();
            // point_corr.z += trans_corr.z();

            pc_out->push_back(point_corr);
            lidar_it++;
        }
    }
    return true;
}