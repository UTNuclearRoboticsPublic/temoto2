#include "ros/ros.h"
#include "temoto_core/common/tools.h"
#include "temoto_core/common/temoto_log_macros.h"
#include "TTP/task_manager.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // Name of the node
    std::string node_name = "temoto_agent";

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = temoto_core::common::generateLogPrefix(node_name, "", __func__);

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
    TTP::TaskManager task_manager(node_name, temoto_core::error::Subsystem::AGENT, true, "", "human_chatter");

    // Publish a message to the tasking core
    std_msgs::String init_msg;
    init_msg.data = temoto_core::common::getTemotoNamespace() + " start the terminal";
    chatter_publisher.publish(init_msg);

    std::cout << prefix << " Core is up and running\n\n";

    // Start the spinner and chill
    ros::AsyncSpinner spinner(0);
    try
    {
      spinner.start();
      ros::waitForShutdown();
    }
    catch(...)
    {
      std::cout << "\n The agent crashed\n";
    }


    return 0;
}
