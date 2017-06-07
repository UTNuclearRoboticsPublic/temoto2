#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/Int32.h"

serial::Serial ser;

void blinkCallback(const std_msgs::Int32 msg)
{
    ROS_INFO("Received message %d", msg.data);

    if ( msg.data == 1 )
    {
        ROS_INFO("Enabling the blinker");
        ser.write("1");
    }

    else
    {
        ROS_INFO("Disabling the blinker");
        ser.write("9");
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "blinker");
    ros::NodeHandle nh;

    ros::Subscriber blinker_subscriber = nh.subscribe("blink", 1000, blinkCallback);
    // ros::Publisher read_pub = nh.advertise<std_msgs::String>("serial_read", 1000);

    try // Open serial communication
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return 1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }

    else
    {
        return 1;
    }

    ros::spin();

}
