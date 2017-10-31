#include "ros/ros.h"
#include "common/tools.h"
#include "common/temoto_log_macros.h"
#include "TTP/tasking_core.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // Name of the node
    std::string node_name = "temoto_agent";

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(node_name, "", __func__);

    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * " << std::endl;
    std::cout << "*                                                     * " << std::endl;
    std::cout << "*                       TEMOTO CORE                   * " << std::endl;
    std::cout << "*                          v.2.0                      * " << std::endl;
    std::cout << "*                                                     * " << std::endl;
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * \n" << std::endl;

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Publisher for publishing messages to core itself
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("/temoto_2/human_chatter", 1000);

    // Create a tasking core
    TTP::TaskingCore tasking_core(node_name);

    // Publish a message to the tasking core
    std_msgs::String init_msg;
    init_msg.data = "start the engine";
    chatter_publisher.publish(init_msg);

    std::cout << prefix << " Core is up and running\n";

    // Start the spinner and chill
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
