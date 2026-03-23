/*
 * separate main function for lio_sphere nodelet to allow for standalone node execution without a nodelet manager
 */

#include "lio_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LIONode node(nh, pnh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}