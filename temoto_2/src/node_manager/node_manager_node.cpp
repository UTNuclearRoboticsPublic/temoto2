#include "ros/ros.h"
#include "node_manager/node_manager.h"

//#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "node_manager");

    // Create instance of node manager
	node_manager::NodeManager nm;

	// set up ROS timer to update the node_manager
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1), &node_manager::NodeManager::update, &nm);
    ros::Rate loop_rate(10);

    ROS_INFO("[node_manager_node/main] Node Manager Node is good to go");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
