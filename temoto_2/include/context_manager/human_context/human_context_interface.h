#include "core/common.h"

#include "base_task/task_errors.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"
#include "temoto_2/gestureSpecifier.h"
#include "temoto_2/speechSpecifier.h"
#include "temoto_2/getGestures.h"
#include "temoto_2/getSpeech.h"
#include "temoto_2/stopAllocatedServices.h"
#include "std_msgs/Float32.h"
#include "human_msgs/Hands.h"

//#include <boost/function.hpp>

template <class T> class HumanContextInterface
{
public:

    HumanContextInterface()
    {
        // Start the clients
        this->get_gestures_client_ = n_.serviceClient<temoto_2::getGestures>("setup_gesture");
        this->get_speech_client_ = n_.serviceClient<temoto_2::getSpeech>("setup_speech");
        this->stop_allocated_services_ = n_.serviceClient<temoto_2::stopAllocatedServices>("stop_allocated_services_hc");
    }

    void getGestures (std::vector <temoto_2::gestureSpecifier> gesture_specifiers, void(T::*callback)(human_msgs::Hands), T* obj)
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        // Contact the "Context Manager", pass the gesture specifier and if successful, get
        // the name of the topic
        temoto_2::getGestures get_gestures_srv;
        get_gestures_srv.request.gesture_specifiers = gesture_specifiers;
        get_gestures_srv.request.id = id_;

        // Call the server
        if (!get_gestures_client_.call(get_gestures_srv))
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        // Check if the request was satisfied
        if (get_gestures_srv.response.code != 0)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Service request failed: " + get_gestures_srv.response.message,
                                         ros::Time::now());
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getGestures] subscribing to topic'%s'", get_gestures_srv.response.topic.c_str());
        gesture_subscriber_ = n_.subscribe(get_gestures_srv.response.topic, 1000, callback, obj);

        // Get the responded id
        id_ = get_gestures_srv.response.id;
    }

    void getSpeech(std::vector <temoto_2::speechSpecifier> speech_specifiers, void(T::*callback)(std_msgs::String), T* obj)
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        // Contact the "Context Manager", pass the speech specifier and if successful, get
        // the name of the topic

        temoto_2::getSpeech get_speech_srv;
        get_speech_srv.request.speech_specifiers = speech_specifiers;
        get_speech_srv.request.id = id_;

        // Call the server
        if (!get_speech_client_.call(get_speech_srv))
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        // Check if the request was satisfied
        if (get_speech_srv.response.code != 0)
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Service request failed: " + get_speech_srv.response.message,
                                         ros::Time::now());
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getSpeech] subscribing to topic'%s'", get_speech_srv.response.topic.c_str());
        speech_subscriber_ = n_.subscribe(get_speech_srv.response.topic, 1000, callback, obj);

        // Get the responded id
        id_ = get_speech_srv.response.id;
    }

    bool stopAllocatedServices()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        temoto_2::stopAllocatedServices stopSrv;
        stopSrv.request.id = id_;

        // Call the server
        if (!stop_allocated_services_.call(stopSrv))
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::CORE,
                                         error::Urgency::HIGH,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }
    }

    ~HumanContextInterface()
    {
        // Let the context manager know, that task is finished and topics are unsubscribed
        stopAllocatedServices();
    }

private:

    TemotoID::ID id_ = TemotoID::UNASSIGNED_ID;

    const std::string class_name_ = "HumanContextInterface";

    ros::NodeHandle n_;
    ros::Subscriber gesture_subscriber_;
    ros::Subscriber speech_subscriber_;

    ros::ServiceClient get_gestures_client_;
    ros::ServiceClient get_speech_client_;
    ros::ServiceClient stop_allocated_services_;
};

