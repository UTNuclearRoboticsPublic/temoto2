#include "ros/ros.h"
#include "std_msgs/String.h"

std::string string_0, string_1;

// Subscriber 0 callback
void sub0Cb(const std_msgs::String &msg)
{
  std::cout << "Changing string_0 to " << msg.data << std::endl;
  string_0 = msg.data;
}

// Subscriber 1 callback
void sub1Cb(const std_msgs::String &msg)
{
  std::cout << "Changing string_1 to " << msg.data << std::endl;
  string_1 = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm_0_node");
  ros::NodeHandle nh;

  // Test publishers
  ros::Publisher pub_0 = nh.advertise<std_msgs::String>("algorithm_0_pub_0", 10);
  ros::Publisher pub_1 = nh.advertise<std_msgs::String>("algorithm_0_pub_1", 10);

  // Test subscribers
  ros::Subscriber sub_0 = nh.subscribe( "algorithm_0_sub_0", 10, sub0Cb);
  ros::Subscriber sub_1 = nh.subscribe( "algorithm_0_sub_1", 10, sub1Cb);

  string_0 = "default";
  string_1 = "default";

  std::cout << "Algorithm_0 READY\n";

  // Publish messages with 1 Hz rate
  ros::Rate spin_rate(1);

  while (ros::ok())
  {
    // Create the messages
    std_msgs::String msg_0, msg_1;
    msg_0.data = "Data 0 = " + string_0 + "; (string_1 = " + string_1 + ")";
    msg_1.data = "Data 1 = " + string_1 + "; (string_0 = " + string_0 + ")";

    // Publish the messages
    pub_0.publish(msg_0);
    pub_1.publish(msg_1);

    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
