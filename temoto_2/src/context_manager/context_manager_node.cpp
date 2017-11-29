#include "ros/ros.h"
#include "std_msgs/String.h"
#include "context_manager/context_manager.h"

#include <sstream>

/*
 * A node that starts all context managers
 */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "context_manager");
    ros::NodeHandle n;

    // Create instance of human context
    context_manager::ContextManager contextManager;

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
