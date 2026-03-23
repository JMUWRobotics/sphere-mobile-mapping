#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "lio_node.hpp"

namespace lio_sphere
{
    class LioNodelet : public nodelet::Nodelet
    {
    public:
        void onInit() override
        {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            node_.reset(new LIONode(nh, pnh));
            NODELET_INFO("lio_sphere nodelet loaded.");
        }

    private:
        std::unique_ptr<LIONode> node_;
    };
} // namespace lio_sphere

PLUGINLIB_EXPORT_CLASS(lio_sphere::LioNodelet, nodelet::Nodelet)