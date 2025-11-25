#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// Publisher
ros::Publisher pub_vis_n;

double dot_product_test(std::vector<double> &v1, std::vector<double> &v2){
    double product = 0.0;
    for(int i=0; i<v1.size(); i++)
        product += v1[i] * v2[i];
    return product;
}

std::vector<double> subtract_points_test(pcl::PointXYZ &p1, pcl::PointXYZ &p2){
    std::vector<double> d = {0.0, 0.0, 0.0};
    d[0] = p1.x - p2.x;
    d[1] = p1.y - p2.y;
    d[2] = p1.z - p2.z;
    return d;
}

void scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    //ROS_INFO("inside callback");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //std::cout << "PointCloud read in: " << temp_cloud->width * temp_cloud->height << " data points." << std::endl;

    for(int i = 0; i < temp_cloud->size(); i++ ){
        pcl::PointXYZI v = temp_cloud->points[i];

        float x = v._PointXYZI::data[0];
        float y = v._PointXYZI::data[1];
        float z = v._PointXYZI::data[2];
        if(i == (temp_cloud->size() - 1)){
            std::cout << "Last point: " << x << "\t" << y << "\t" << z << std::endl;
        }
    }

    //std::vector<pcl::PointXYZI> data = temp_cloud->points;
    //ROS_INFO(data);

}

void vis_marker(const sensor_msgs::PointCloud2ConstPtr& msg){
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = msg->header.stamp;
    // marker.ns = "normal_vectors";
    // marker.id = 000;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = query_point.x;
    // marker.pose.position.y = query_point.y;
    // marker.pose.position.z = query_point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    std::vector<double> n = {0.33, 0.66, -0.66}; // Previous normalized normal vector (n)
    geometry_msgs::Point start, end;
    start.x = start.y = start.z = 0.0;
    end.x = (start.x + n[0])/3.0;
    end.y = (start.y + n[1])/3.0;
    end.z = (start.z + n[2])/3.0;
    marker.points = {start, end};

    marker.scale.x = 0.02;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Publish marker
    pub_vis_n.publish(marker);

}

int main(int argc, char** argv){
    /* Init node */
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    printf("\n--------------------------------------\n\tSTARTING TEST!\n--------------------------------------\n");

    // pub_vis_n = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    // /* Subscribe to Hesai laser scanner */
    // ros::Subscriber sub;
    // sub = nh.subscribe("/hesai/pandar", 1, vis_marker);

    // pcl::PointXYZ pc = {1.0, 1.0, 4.2};
    // pcl::PointXYZ pi = {3.0, 2.0, 3.5};

    // std::vector<double> d = subtract_points_test(pi, pc);
    // std::vector<double> n = {0.5, 1.0, -1.0};

    // ROS_INFO("Subtraction: (%.3f, %.3f, %.3f) \n", d[0], d[1], d[2]);
    // ROS_INFO("Dotproduct: %.3f", dot_product_test(d, n));

    double test = 0.0;

    auto start = std::chrono::high_resolution_clock::now();
    test = pow(3.0,2) + pow(2.5,2) + pow(4.1,2);
    test = pow(3.0,2) + pow(2.5,2) + pow(4.1,2);
    test = pow(3.0,2) + pow(2.5,2) + pow(4.1,2);
    test = pow(3.0,2) + pow(2.5,2) + pow(4.1,2);
    test = pow(3.0,2) + pow(2.5,2) + pow(4.1,2);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    ROS_INFO("sqr(x) with pow takes: %ld", duration);

    start = std::chrono::high_resolution_clock::now();
    test = 3.0 * 3.0 + 2.5 * 2.5 + 4.1 * 4.1;
    test = 3.0 * 3.0 + 2.5 * 2.5 + 4.1 * 4.1;
    test = 3.0 * 3.0 + 2.5 * 2.5 + 4.1 * 4.1;
    test = 3.0 * 3.0 + 2.5 * 2.5 + 4.1 * 4.1;
    test = 3.0 * 3.0 + 2.5 * 2.5 + 4.1 * 4.1;
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    ROS_INFO("sqr(x) with mul takes: %ld", duration);

    ros::Rate freq(50);
    /* Loop */
    ros::spin();
    freq.sleep();
}
