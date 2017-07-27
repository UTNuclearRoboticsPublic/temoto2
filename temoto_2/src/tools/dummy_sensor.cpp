/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* The purpose of this node is to publish meaningless data
* that can be used for debugging purposes.
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_sensor");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher data_pub = n.advertise<std_msgs::Float32>("dummy_sensor_data", 100);

    float counter = 0;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::Float32 msg;
        msg.data = sin(counter);
        counter += 0.02;

        ROS_INFO("sending: %f", msg.data);
        data_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
