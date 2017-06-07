#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "language_input");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("human_chatter", 1000);

    while (ros::ok())
    {
      std_msgs::String msg;
      std::string chatter;

      std::getline(std::cin, chatter);
      msg.data = chatter;

      ROS_INFO("sending: %s", msg.data.c_str());

      chatter_pub.publish(msg);

      ros::spinOnce();
    }

    return 0;
}
