#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_0_node");
  ros::NodeHandle nh;

  // Test publishers
  ros::Publisher pub_0 = nh.advertise<std_msgs::String>("sensor_0_pub_0", 10);
  ros::Publisher pub_1 = nh.advertise<std_msgs::String>("sensor_0_pub_1", 10);

  std::cout << "Sensor_0 READY\n";

  // Publish messages with 1 Hz rate
  ros::Rate spin_rate(1);

  // Some data
  unsigned int i = 0;

  while (ros::ok())
  {
    // Create the messages
    std_msgs::String msg_0, msg_1;
    msg_0.data = "(sensor_0/topic_0 DATA = " + std::to_string(i) + ")";
    msg_1.data = "(sensor_0/topic_1 DATA = " + std::to_string(i) + ")";

    i++;

    // Publish the messages
    pub_0.publish(msg_0);
    pub_1.publish(msg_1);

    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
