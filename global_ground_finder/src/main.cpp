/*
 * main.cpp
 * Main entry point for global ground finder node
 * Validates parameters and initializes the GlobalGroundFinder class
 *
 * Based on main.cpp by Carolin Bösch
 */

#include "global_ground_finder.h"

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "global_ground_finder");
    printf("\n-----------------------------------------------\n\t\tSTARTING GLOBAL GROUND FINDER!\n-----------------------------------------------\n");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PlaneSegm plane_alg;

    bool quiet;
    if (!pnh.getParam("quiet", quiet))
    {
        quiet = false; // Default value
    }
    else
    {
        if (quiet)
        {
            ROS_INFO("Quiet Mode activated! Repetitive messages will be suppressed. Start-up information, errors and warnings will be shown!");
        }
    }

    std::string plane_alg_str;
    if (!pnh.getParam("plane_algorithm", plane_alg_str))
    {
        ROS_WARN("No plane_algorithm parameter specified, defaulting to 'ransac'");
        plane_alg = RANSAC;
    }
    else
    {
        ROS_INFO("Using plane segmentation algorithm: %s", plane_alg_str.c_str());
        if (!strcmp(plane_alg_str.c_str(), "pca"))
        {
            plane_alg = PCA;
        }
        else if (!strcmp(plane_alg_str.c_str(), "ransac"))
        {
            plane_alg = RANSAC;
        }
        else if (!strcmp(plane_alg_str.c_str(), "rht"))
        {
            plane_alg = RHT;
        }
        else if (!strcmp(plane_alg_str.c_str(), "rht2"))
        {
            plane_alg = RHT2;
        }
        else
        {
            ROS_ERROR("Invalid Parameter plane_algorithm='%s'! Valid options are: 'pca', 'ransac', 'rht', 'rht2'. Ending Node!", plane_alg_str.c_str());
            ros::shutdown();
            return 1;
        }
    }

    // File logging param
    std::string log_filename;
    if (!pnh.getParam("log_file", log_filename))
    {
        ROS_INFO("No log_file parameter specified. CSV logging disabled.");
    }
    else if (log_filename != "default")
    {
        ROS_INFO("CSV logging enabled: %s", log_filename.c_str());
    }

    GlobalGroundFinder global_ground_finder(nh, pnh, plane_alg);

    ROS_INFO("Global Ground Finder node running...");
    ROS_INFO("Waiting for global map on /map_out and pose on /lkf/pose");

    // Loop
    ros::Rate freq(50);
    ros::spin();
    freq.sleep();

    return 0;
}
