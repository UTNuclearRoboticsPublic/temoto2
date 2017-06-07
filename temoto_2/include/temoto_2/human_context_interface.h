#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <boost/function.hpp>

template <class T> class Human_context_interface
{
public:

    int getGestures( void(T::*callback)(std_msgs::String), T* obj)
    {
        ROS_INFO("[Human_context_interface] subscribing to /gestures topic");
        gestureSubscriber_ = n_.subscribe("gestures", 1000, callback, obj);

        return 0;
    }

    int getVoiceCommands( void(T::*callback)(std_msgs::String), T* obj)
    {
        ROS_INFO("[Human_context_interface] subscribing to /gestures topic");
        voiceCommandSubscriber_ = n_.subscribe("gestures", 1000, callback, obj);

        return 0;
    }

    ~Human_context_interface()
    {
        //gestureSubscriber_.shutdown();
    }

private:

    ros::NodeHandle n_;
    ros::Subscriber gestureSubscriber_;
    ros::Subscriber voiceCommandSubscriber_;
};

