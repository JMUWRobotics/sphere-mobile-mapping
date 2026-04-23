#include <cstring>
#include <memory>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/package.h>

#include "global_ground_finder.h"

/*
 * This file replaces main.cpp in the context of the nodelets manager. If nodelet is used as standalone, main.cpp can be used.
 */

namespace global_ground_finder
{
    namespace
    {
        bool ensureDirExists(const std::string &path)
        {
            struct stat info;
            if (::stat(path.c_str(), &info) == 0)
            {
                return S_ISDIR(info.st_mode);
            }

            if (::mkdir(path.c_str(), 0755) == 0)
            {
                return true;
            }

            if (errno == EEXIST)
            {
                return true;
            }

            return false;
        }
    } // namespace

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
                plane_alg_str = "ransac";
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

            const std::string package_path = ros::package::getPath("global_ground_finder");
            const std::string data_dir = package_path + "/data";
            const std::string algo_dir = data_dir + "/" + plane_alg_str;

            if (!ensureDirExists(data_dir))
            {
                NODELET_ERROR("Failed to ensure data directory exists: %s", data_dir.c_str());
                return;
            }
            if (!ensureDirExists(algo_dir))
            {
                NODELET_ERROR("Failed to ensure algorithm data directory exists: %s", algo_dir.c_str());
                return;
            }

            // File logging param
            std::string log_filename;
            if (!pnh.getParam("file", log_filename) && !pnh.getParam("log_file", log_filename))
            {
                NODELET_INFO("No file parameter specified. CSV logging disabled.");
            }
            else if (log_filename != "default")
            {
                NODELET_INFO("CSV logging enabled: %s", log_filename.c_str());
            }

            node_.reset(new ::GlobalGroundFinder(nh, pnh, plane_alg));
            NODELET_INFO("Global Ground Finder node running...");
            NODELET_INFO("Waiting for registered poses by lio_node on /all_pose_out");
            NODELET_INFO("Global Ground Finder nodelet loaded.");
        }

    private:
        std::unique_ptr<::GlobalGroundFinder> node_;
    };
} // namespace global_ground_finder

PLUGINLIB_EXPORT_CLASS(global_ground_finder::GlobalGroundFinderNodelet, nodelet::Nodelet)