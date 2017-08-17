#include "ros/ros.h"
#include "std_msgs/String.h"
#include "output_manager/rviz_manager/rviz_manager.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "output_manager");
    ros::NodeHandle n;

    // Create instance of human context
    RvizManager rviz_manager;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
