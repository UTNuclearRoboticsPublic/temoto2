#include "ros/ros.h"
#include "std_msgs/String.h"
#include "temoto_error/temoto_error.h"
#include "common/console_colors.h"

void displayErrorMessages( const temoto_2::ErrorStack &e_stack )
{
    error::ErrorStack e = e_stack.errorStack;
    std::cout << e << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "language_input");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/terminal_text", 1000);

    // Subscribe to error messages
    ros::Subscriber error_subscriber = n.subscribe( "/temoto_2/temoto_error_messages", 100, displayErrorMessages);

    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * *" << std::endl;
    std::cout << "*                                                     *" << std::endl;
    std::cout << "*                     TEMOTO TERMINAL                 *" << std::endl;
    std::cout << "*                         v.1.0                       *" << std::endl;
    std::cout << "*                                                     *" << std::endl;
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * *" << std::endl;
    std::cout << "\n\n";

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (ros::ok())
    {
      std_msgs::String msg;
      std::string chatter;

      std::cout << ":$ ";
      std::getline(std::cin, chatter);
      msg.data = chatter;

      //ROS_INFO("sending: %s", msg.data.c_str());

      chatter_pub.publish(msg);
    }

    return 0;
}
