#include "ros/ros.h"
#include "temoto_context_manager/context_manager.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temoto_context_manager");
    ros::NodeHandle n;

    // Create instance of ContextManager
    temoto_context_manager::ContextManager context_manager;

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
