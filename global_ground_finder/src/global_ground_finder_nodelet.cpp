#include <cstring>
#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "global_ground_finder.h"

/*
 * This file replaces main.cpp in the context of the nodelets manager. If nodelet is used as standalone, main.cpp can be used.
 */

namespace global_ground_finder
{
    class GlobalGroundFinderNodelet : public nodelet::Nodelet
    {
    public:
        void onInit() override // override nodelet onInit function to initialize new GlobalGroundFinder object with set params
        {
            NODELET_INFO("\n-----------------------------------------------\n\t\tSTARTING GLOBAL GROUND FINDER!\n-----------------------------------------------\n");
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

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
                    NODELET_INFO("Quiet Mode activated! Repetitive messages will be suppressed. Start-up information, errors and warnings will be shown!");
                }
            }

            std::string plane_alg_str;
            if (!pnh.getParam("plane_algorithm", plane_alg_str))
            {
                NODELET_WARN("No plane_algorithm parameter specified, defaulting to 'ransac'");
                plane_alg = RANSAC;
            }
            else
            {
                NODELET_INFO("Using plane segmentation algorithm: %s", plane_alg_str.c_str());
                if (!std::strcmp(plane_alg_str.c_str(), "pca"))
                {
                    plane_alg = PCA;
                }
                else if (!std::strcmp(plane_alg_str.c_str(), "ransac"))
                {
                    plane_alg = RANSAC;
                }
                else if (!std::strcmp(plane_alg_str.c_str(), "rht"))
                {
                    plane_alg = RHT;
                }
                else if (!std::strcmp(plane_alg_str.c_str(), "rht2"))
                {
                    plane_alg = RHT2;
                }
                else
                {
                    NODELET_ERROR("Invalid parameter plane_algorithm='%s'. Valid options are: 'pca', 'ransac', 'rht', 'rht2'.", plane_alg_str.c_str());
                    return;
                }
            }

            // File logging param
            std::string log_filename;
            if (!pnh.getParam("log_file", log_filename))
            {
                NODELET_INFO("No log_file parameter specified. CSV logging disabled.");
            }
            else if (log_filename != "default")
            {
                NODELET_INFO("CSV logging enabled: %s", log_filename.c_str());
            }

            node_.reset(new ::GlobalGroundFinder(nh, pnh, plane_alg));
            NODELET_INFO("Global Ground Finder node running...");
            NODELET_INFO("Waiting for global map on /map_out and pose on /lkf/pose");
            NODELET_INFO("Global Ground Finder nodelet loaded.");
        }

    private:
        std::unique_ptr<::GlobalGroundFinder> node_;
    };
} // namespace global_ground_finder

PLUGINLIB_EXPORT_CLASS(global_ground_finder::GlobalGroundFinderNodelet, nodelet::Nodelet)