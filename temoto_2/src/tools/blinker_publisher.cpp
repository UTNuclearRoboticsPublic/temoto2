#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blinker_publisher");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("blink", 1000);

    while (ros::ok())
    {
      std_msgs::Int32 msg;

      std::cin >> msg.data;

      ROS_INFO("sending: %d", msg.data);

      chatter_pub.publish(msg);

      ros::spinOnce();
    }

    return 0;
}
