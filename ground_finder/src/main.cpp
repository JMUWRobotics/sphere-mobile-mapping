/*
 * main.cpp
 * Calling the GroundFinder class according to set params
 *
 * PARAMS: //TODO
 *
 * Author: Carolin Bösch
 */

#include "ground_finder.h"

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "ground_finder");
    printf("\n-----------------------------------------------\n\t\tSTARTING GROUND FINDER!\n-----------------------------------------------\n");
    ros::NodeHandle nh;

    Preprocessing filtering;
    Preprocessing subcloud;
    PlaneSegm plane_alg;

    // Quiet param
    bool quiet;
    if (!nh.getParam("/ground_finder_node/quiet", quiet))
    {
        ROS_ERROR("Parameter failure... Ending Node!");
        ros::shutdown();
    }
    else
    {
        if (quiet)
        {
            ROS_INFO("Quiet Mode activated! No receiving data or repetetive messages will be shown. Start-up information, errors and warnings will be shown!!");
        }
    }

    // Filter param (valid: "none", "geo" or "kdt")
    std::string filtering_str;
    if (!nh.getParam("/ground_finder_node/filter", filtering_str))
    {
        ROS_ERROR("Parameter failure... Ending Node!");
        ros::shutdown();
    }
    else
    {
        ROS_WARN("Approach used for filtering: %s", filtering_str.c_str());
        if (!strcmp(filtering_str.c_str(), "none"))
        {
            filtering = NONE;
        }
        else if (!strcmp(filtering_str.c_str(), "geo"))
        {
            filtering = GEOMETRICAL;
        }
        else if (!strcmp(filtering_str.c_str(), "kdt"))
        {
            filtering = KD_TREE;
        }
        else
        {
            ROS_ERROR("Invalid Parameter filter! Valid options are: 'none', 'geo' or 'kdt'. Ending Node!");
            ros::shutdown();
        }
    }

    // Subcloud param (valid: "geo" or "kdt")
    std::string subcloud_str;
    if (!nh.getParam("/ground_finder_node/subcloud", subcloud_str))
    {
        ROS_ERROR("Parameter failure... Ending Node!");
        ros::shutdown();
    }
    else
    {
        ROS_WARN("Approach used for subcloud: %s", subcloud_str.c_str());
        if (!strcmp(subcloud_str.c_str(), "geo"))
        {
            subcloud = GEOMETRICAL;
        }
        else if (!strcmp(subcloud_str.c_str(), "kdt"))
        {
            subcloud = KD_TREE;
        }
        else
        {
            ROS_ERROR("Invalid Parameter subcloud! Valid options are: 'geo' or 'kdt'. Ending Node!");
            ros::shutdown();
        }
    }

    // Plane segmentation param (valid: "lsf", "pca", "ran" or "rht")
    std::string plane_alg_str;
    if (!nh.getParam("/ground_finder_node/plane", plane_alg_str))
    {
        ROS_ERROR("Parameter failure... Ending Node!");
        ros::shutdown();
    }
    else
    {
        ROS_WARN("Using plane segmentation alg.: %s", plane_alg_str.c_str());
        if (!strcmp(plane_alg_str.c_str(), "lsf"))
        {
            plane_alg = LSF;
        }
        else if (!strcmp(plane_alg_str.c_str(), "pca"))
        {
            plane_alg = PCA;
        }
        else if (!strcmp(plane_alg_str.c_str(), "ran"))
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
            ROS_ERROR("Invalid Parameter type! Valid options are: 'lsf', 'pca', 'ran' or 'rht'. Ending Node!");
            ros::shutdown();
        }
    }

    // File param
    std::string filename;
    std::string path = "";
    bool write2file = false;
    if (!nh.getParam("/ground_finder_node/file", filename))
    {
        ROS_ERROR("Parameter failure... Ending Node!");
        ros::shutdown();
    }
    else
    {
        if (strcmp(filename.c_str(), "default"))
        {
            write2file = true;
            if (filtering == GEOMETRICAL && subcloud == GEOMETRICAL)
            {
                filename = "geo-geo-" + filename + ".csv";
            }
            else if (filtering == GEOMETRICAL && subcloud == KD_TREE)
            {
                filename = "geo-kdt-" + filename + ".csv";
            }
            else if (filtering == KD_TREE && subcloud == GEOMETRICAL)
            {
                filename = "kdt-geo-" + filename + ".csv";
            }
            else if (filtering == KD_TREE && subcloud == KD_TREE)
            {
                filename = "kdt-kdt-" + filename + ".csv";
            }
            else if (filtering == NONE && subcloud == GEOMETRICAL)
            {
                filename = "none-geo-" + filename + ".csv";
            }
            else if (filtering == NONE && subcloud == KD_TREE)
            {
                filename = "none-kdt-" + filename + ".csv";
            }

            switch (plane_alg)
            {
            case LSF:
                path = "/home/t2/catkin_ws/src/ground_finder/data/lsf/" + filename;
                break;
            case PCA:
                path = "/home/t2/catkin_ws/src/ground_finder/data/pca/" + filename;
                break;
            case RANSAC:
                path = "/home/t2/catkin_ws/src/ground_finder/data/ran/" + filename;
                break;
            case RHT:
                path = "/home/t2/catkin_ws/src/ground_finder/data/rht/" + filename;
                break;
            case RHT2:
                path = "/home/t2/catkin_ws/src/ground_finder/data/rht2/" + filename;
                break;
            }
            ROS_WARN("Writing to file: %s\n", path.c_str());
        }
        else
        {
            ROS_INFO("No file name specified. Not writing to csv file!");
        }
    }

    GroundFinder ground_finder(filtering, subcloud, plane_alg, quiet, write2file, path);

    // Loop
    ros::Rate freq(50);
    ros::spin();
    freq.sleep();

    return 0;
}
