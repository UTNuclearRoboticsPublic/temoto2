#include "core/common.h"

#include "temoto_2/gestureSpecifier.h"
#include "temoto_2/speechSpecifier.h"
#include "temoto_2/getGestures.h"
#include "temoto_2/getSpeech.h"
#include "temoto_2/stopAllocatedServices.h"
#include "std_msgs/Float32.h"

//#include <boost/function.hpp>

template <class T> class HumanContextInterface
{
public:

    HumanContextInterface()
    {
        // Start the clients
        this->getGesturesClient_ = n_.serviceClient<temoto_2::getGestures>("setup_gesture");
        this->getSpeechClient_ = n_.serviceClient<temoto_2::getSpeech>("setup_speech");
        this->stopAllocatedServices_ = n_.serviceClient<temoto_2::stopAllocatedServices>("stop_allocated_services");
    }

    bool getGestures (std::vector <temoto_2::gestureSpecifier> gestureSpecifiers, void(T::*callback)(std_msgs::Float32), T* obj)
    {
        // Contact the "Context Manager", pass the gesture specifier and if successful, get
        // the name of the topic

        temoto_2::getGestures getGesturesSrv;
        getGesturesSrv.request.gestureSpecifiers = gestureSpecifiers;
        getGesturesSrv.request.id = id_;

        // Call the server
        while (!getGesturesClient_.call(getGesturesSrv))
        {
            ROS_ERROR("[HumanContextInterface::getGestures] Failed to call service, trying again...");
        }

        // Check if the request was satisfied
        if (getGesturesSrv.response.code != 0)
        {
            ROS_ERROR("[HumanContextInterface::getGestures] /setup_gesture request failed: %s", getGesturesSrv.response.message.c_str());
            return false;
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getGestures] subscribing to topic'%s'", getGesturesSrv.response.topic.c_str());
        gestureSubscriber_ = n_.subscribe(getGesturesSrv.response.topic, 1000, callback, obj);

        // Get the responded id
        id_ = getGesturesSrv.response.id;

        return true;
    }

    bool getSpeech(std::vector <temoto_2::speechSpecifier> speechSpecifiers, void(T::*callback)(std_msgs::String), T* obj)
    {
        // Contact the "Context Manager", pass the speech specifier and if successful, get
        // the name of the topic

        temoto_2::getSpeech getSpeechSrv;
        getSpeechSrv.request.speechSpecifiers = speechSpecifiers;
        getSpeechSrv.request.id = id_;

        // Call the server
        while (!getSpeechClient_.call(getSpeechSrv))
        {
            ROS_ERROR("[HumanContextInterface::getSpeech] Failed to call service, trying again...");
        }

        // Check if the request was satisfied
        if (getSpeechSrv.response.code != 0)
        {
            ROS_ERROR("[HumanContextInterface::getSpeech] /setup_speech request failed: %s", getSpeechSrv.response.message.c_str());
            return false;
        }

        // Subscribe to the topic that was provided by the "Context Manager"
        ROS_INFO("[HumanContextInterface::getSpeech] subscribing to topic'%s'", getSpeechSrv.response.topic.c_str());
        speechSubscriber_ = n_.subscribe(getSpeechSrv.response.topic, 1000, callback, obj);

        // Get the responded id
        id_ = getSpeechSrv.response.id;

        return true;
    }

    bool stopAllocatedServices()
    {
        temoto_2::stopAllocatedServices stopSrv;
        stopSrv.request.id = id_;

        // Call the server
        while (!stopAllocatedServices_.call(stopSrv))
        {
            ROS_ERROR("[HumanContextInterface::stopAllocatedServices] Failed to call service, trying again...");
        }

        return true;
    }

    ~HumanContextInterface()
    {
        // Let the context manager know, that task is finished and topics are unsubscribed
        stopAllocatedServices();
    }

private:

    std::string id_;

    //ros::NodeHandlePtr n_ = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandle n_;
    ros::Subscriber gestureSubscriber_;
    ros::Subscriber speechSubscriber_;

    ros::ServiceClient getGesturesClient_;
    ros::ServiceClient getSpeechClient_;
    ros::ServiceClient stopAllocatedServices_;
};

