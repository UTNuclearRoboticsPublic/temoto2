#ifndef HUMAN_CONTEXT_INTERFACE_H
#define HUMAN_CONTEXT_INTERFACE_H

#include "core/common.h"

#include "base_task/task_errors.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"

#include "std_msgs/Float32.h"
#include "human_msgs/Hands.h"

#include "context_manager/human_context/human_context_services.h"
#include "rmp/resource_manager.h"
#include <vector>

//#include <boost/function.hpp>

template <class T> class HumanContextInterface
{
public:

    HumanContextInterface() : resource_manager_("human_context_interface", this)
    {
    }

    void getGestures (std::vector <temoto_2::GestureSpecifier> gesture_specifiers, void(T::*callback)(human_msgs::Hands), T* obj)
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

        // Contact the "Context Manager", pass the gesture specifier and if successful, get
        // the name of the topic
        temoto_2::LoadGesture srv_msg;
        srv_msg.request.gesture_specifiers = gesture_specifiers;

        // Call the server
		try
		{
        resource_manager_.template call<temoto_2::LoadGesture>(
				human_context::srv_name::MANAGER,
				human_context::srv_name::GESTURE_SERVER,
				srv_msg);
		}
		catch(...)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        // Check if the request was satisfied
		// TODO: in future, catch code==0 exeption from RMP and rethrow from here
        if (srv_msg.response.rmp.code != 0)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Service request failed: " + srv_msg.response.rmp.message,
                                         ros::Time::now());
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getGestures] subscribing to topic'%s'", srv_msg.response.topic.c_str());
        gesture_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, callback, obj);
    }


    void getSpeech(std::vector <temoto_2::SpeechSpecifier> speech_specifiers, void(T::*callback)(std_msgs::String), T* obj)
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

        // Contact the "Context Manager", pass the speech specifier and if successful, get
        // the name of the topic

        temoto_2::LoadSpeech srv_msg;
        srv_msg.request.speech_specifiers = speech_specifiers;

        // Call the server
		try
		{
            ROS_INFO("getting speech");
        resource_manager_.template call<temoto_2::LoadSpeech>(
				human_context::srv_name::MANAGER,
				human_context::srv_name::SPEECH_SERVER,
				srv_msg);
		}
		catch(...)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        // Check if the request was satisfied
		// TODO: in future, catch code==0 exeption from RMP and rethrow from here
        if (srv_msg.response.rmp.code != 0)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Service request failed: " + srv_msg.response.rmp.message,
                                         ros::Time::now());
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getSpeech] subscribing to topic'%s'", srv_msg.response.topic.c_str());
        speech_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, callback, obj);
    }

    bool stopAllocatedServices()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

		try
		{
			// remove all connections, which were created via call() function
			resource_manager_.unloadClients();
		}
		catch(...)
		{
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::CORE,
                                         error::Urgency::HIGH,
                                         prefix + " Failed to unload resources",
                                         ros::Time::now());
		}
    }

    ~HumanContextInterface()
    {
        // Let the context manager know, that task is finished and topics are unsubscribed
        stopAllocatedServices();
    }

    const std::string& getName() const
    {
      return class_name_;
    }

private:

	rmp::ResourceManager<HumanContextInterface> resource_manager_;

    const std::string class_name_ = "human_context_interface";

    ros::NodeHandle nh_;
    ros::Subscriber gesture_subscriber_;
    ros::Subscriber speech_subscriber_;

};

#endif
