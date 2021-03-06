#include "ros/ros.h"
#include "ros/package.h"
#include "common/tools.h"
#include "common/temoto_log_macros.h"
#include "TTP/task_manager.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // Name of the node
    std::string node_name = "resource_snooper";

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(node_name, "", __func__);

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Publisher for publishing messages to core itself
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("resource_snooper_chatter", 1000);

    // Create a tasking core and enable the language processor
    TTP::TaskManager task_manager( node_name
                                  , error::Subsystem::RESOURCE_SNOOPER
                                  , true
                                  , ros::package::getPath(ROS_PACKAGE_NAME) + "/../actions/resource_snooper_actions"
                                  , "resource_snooper_chatter" );

    // Publish a message to the tasking core
//    std_msgs::String init_msg;
//    init_msg.data = "start the terminal";
//    chatter_publisher.publish(init_msg);

    std::cout << prefix << " The Resource Snooper is up and running\n\n";

    // Start the spinner and chill
    ros::AsyncSpinner spinner(0);
    try
    {
      spinner.start();
      ros::waitForShutdown();
    }
    catch(...)
    {
      std::cout << "\n The Resource Snooper crashed\n";
    }


    return 0;
}
