#include "ros/ros.h"
#include "common/tools.h"
#include "common/temoto_log_macros.h"
#include "TTP/task_manager.h"
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
    std::cout << "*                         v.2.0.0                     * " << std::endl;
    std::cout << "*                                                     * " << std::endl;
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * \n" << std::endl;

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Publisher for publishing messages to core itself
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("human_chatter", 1000);

    // Create a tasking core and enable the language processor
    TTP::TaskManager task_manager(node_name, true);

    // Publish a message to the tasking core
    std_msgs::String init_msg;
    init_msg.data = "start the terminal";
    chatter_publisher.publish(init_msg);

    std::cout << prefix << " Core is up and running\n\n";

    // Start the spinner and chill
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
