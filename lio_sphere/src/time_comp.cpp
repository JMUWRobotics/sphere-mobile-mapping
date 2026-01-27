#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <random>

// Function to generate random Eigen::Vector3d vectors within a given range
Eigen::Vector3d generate_random_vector(double min = -10.0, double max = 10.0) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    
    return Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
}

// Function to generate random tf2::Vector3 vectors within a given range
tf2::Vector3 generate_random_tf2_vector(double min = -10.0, double max = 10.0) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    
    return tf2::Vector3(dis(gen), dis(gen), dis(gen));
}

inline Eigen::Vector3d lerp_eigen(const Eigen::Vector3d& start, const Eigen::Vector3d& diff, double t) {
    return start + t * (diff);
}
inline tf2::Vector3 lerp_tf2(const tf2::Vector3& start, const tf2::Vector3& diff, double t) {
    return start + t * (diff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vector_interpolation_benchmark");
    ros::NodeHandle nh;

    // Define two vectors for interpolation
    Eigen::Vector3d eigen_start(1.0, 2.0, 3.0);
    Eigen::Vector3d eigen_end(4.0, 5.0, 6.0);
    tf2::Vector3 tf2_start(1.0, 2.0, 3.0);
    tf2::Vector3 tf2_end(4.0, 5.0, 6.0);

    // Number of iterations to benchmark
    const int iterations = 10000000000000;
    double t = 0.5; // Interpolation factor (for example, halfway point)

    // Benchmark Eigen::Vector3d lerp
    Eigen::Vector3d result_g(0,0,0);
    Eigen::Vector3d add(0,0,0);
    auto start_eigen = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d diff = eigen_end - eigen_start;
    for (int i = 0; i < iterations; ++i) {
        add = Eigen::Vector3d(0,0,1);
        eigen_start += add;
        Eigen::Vector3d result = lerp_eigen(eigen_start, diff, t);
        result_g += result;
    }
    auto end_eigen = std::chrono::high_resolution_clock::now();
    ROS_INFO_STREAM(result_g);
    std::chrono::duration<double> eigen_duration = end_eigen - start_eigen;
    ROS_INFO("Eigen::Vector3d lerp time: %f seconds", eigen_duration.count());

    // Benchmark tf2::Vector3 lerp
    tf2::Vector3 result_tf(0,0,0);
    auto start_tf2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        tf2_start+=tf2::Vector3(0,0,1);
        tf2::Vector3 result = tf2_start.lerp(tf2_end, t);
        result_tf += result;
    }
    auto end_tf2 = std::chrono::high_resolution_clock::now();
    ROS_INFO_STREAM(result_tf);
    std::chrono::duration<double> tf2_duration = end_tf2 - start_tf2;
    ROS_INFO("tf2::Vector3 lerp time: %f seconds", tf2_duration.count());

    auto start_tf2_man = std::chrono::high_resolution_clock::now();
    tf2::Vector3 tf2_diff = tf2_end - tf2_start;
    for (int i = 0; i < iterations; ++i) {
        tf2::Vector3 result = lerp_tf2(tf2_start, tf2_diff, t);
    }
    auto end_tf2_man = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tf2_duration_man = end_tf2_man - start_tf2_man;
    ROS_INFO("tf2::Vector3 man lerp time: %f seconds", tf2_duration_man.count());

    return 0;
}
