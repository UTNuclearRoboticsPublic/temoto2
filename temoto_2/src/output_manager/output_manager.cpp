#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "context_manager/human_context/human_context.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "output_manager");
    ros::NodeHandle n;

    // Create instance of human context
    HumanContext humanContext;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
