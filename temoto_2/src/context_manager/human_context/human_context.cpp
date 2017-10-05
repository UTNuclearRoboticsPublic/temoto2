/*
 * TODO: * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 *       * Decide wether its sensor manager's problem to look if different tasks
 *         are using the same resource or not. Its not a good idea to stop a node
 *         that is used by multiple tasks
 *       * START USING ROS NAMING CONVETIONS
 */

#include "context_manager/human_context/human_context.h"

namespace human_context
{

HumanContext::HumanContext () : resource_manager_("temoto_2/human_context",this)
{
    // Fire up servers
    resource_manager_.addServer<temoto_2::LoadGesture>(
            srv_name::GESTURE_SERVER, 
            &HumanContext::loadGestureCb,
            &HumanContext::unloadGestureCb);
    
    resource_manager_.addServer<temoto_2::LoadSpeech>(
            srv_name::SPEECH_SERVER, 
            &HumanContext::loadSpeechCb,
            &HumanContext::unloadSpeechCb);

    ROS_INFO("[HumanContext::HumanContext] all services and clients initialized, Human Context is good to go.");
}


void HumanContext::loadGestureCb (temoto_2::LoadGesture::Request& req,
                                  temoto_2::LoadGesture::Response& res)
{
    std::string prefix = "[HumanContext::loadGestureCb]:";
    ROS_INFO("%s Gesture requested ...", prefix.c_str());

    temoto_2::LoadSensor msg;
    msg.request.sensor_type = req.gesture_specifiers[0].type;

    // Call the sensor manager to arrange us a gesture sensor
    try
    {
        resource_manager_.call<temoto_2::LoadSensor>(
                sensor_manager::srv_name::MANAGER,
                sensor_manager::srv_name::SERVER,
                msg);
    }
    catch(...)
    {
        ROS_ERROR("%s Service call failed.", prefix.c_str());
        return; //TODO: throw here
    }

    ROS_INFO("%s Got a response: '%s'", prefix.c_str(), msg.response.rmp.message.c_str());
    res.topic = msg.response.topic;
    res.package_name = msg.response.package_name;
    res.executable = msg.response.executable;
    res.rmp.code = msg.response.rmp.code;
    res.rmp.message = "Gesture request was ";
    res.rmp.message =+ (msg.response.rmp.code == 0) ? "satisfied." : "not satisfied.";
    //TODO: send a reasonable response message and code

}


void HumanContext::unloadGestureCb(temoto_2::LoadGesture::Request& req,
                                   temoto_2::LoadGesture::Response& res)
{
    std::string prefix = "[HumanContext::unloadGestureCb]:";
	ROS_INFO("%s Unloading a gesture or something.", prefix.c_str());
}


void HumanContext::loadSpeechCb (temoto_2::LoadSpeech::Request& req,
                                 temoto_2::LoadSpeech::Response& res)
{
//    ROS_INFO("[HumanContext::setup_speech_cb] Received a speech setup request with specifiers[0]: '%s', '%s', '%s'",
//			req.speech_specifiers[0].type.c_str(), req.speech_specifiers[0].package.c_str(), req.speech_specifiers[0].executable.c_str());
//
//    // Check the id of the req. If there is none (the first call from a task) then provide one
//
//    for (auto& activeReq : setupSpeechActive_)
//    {
//        if (compareSpeechRequest(req, activeReq.request))
//        {
//            ROS_INFO("[HumanContext::setup_speech_cb] Same request already available.");
//            res = activeReq.response;
//            res.id = static_cast<int>(id_local);
//            return true;
//        }
//    }
//
//    // The request was not in "setupSpeechActive_" list. Make a new sensor request
//    ROS_INFO("[HumanContext::setup_speech_cb] This request is unique. Setting up the sensor ...");
//
//    temoto_2::LoadSensor msg;
//    msg.request.sensor_type = req.speech_specifiers[0].type;
//
//    // Call the sensor manager
//    try
//    {
//        resource_manager_.call<temoto_2::LoadSensor>(
//                sensor_manager::srv_name::MANAGER,
//                sensor_manager::srv_name::SERVER,
//                msg);
//    }
//    catch(...)
//    {
//        ROS_ERROR("[HumanContext::setup_speech_cb] Failed to call service /start_sensor, trying again...");
//    }
//
//    ROS_INFO("[HumanContext::setup_speech_cb] Got a response from service /start_sensor: '%s'", msg.response.rmp.message.c_str());
//    res.id = static_cast<int>(id_local);
//    res.topic = msg.response.topic;
//    res.name = msg.response.package_name;
//    res.executable = msg.response.executable;
//    res.code = msg.response.rmp.code;
//
//    // Check the response message, if all is ok then
//    if (msg.response.rmp.code == 0)
//    {   
//        res.message = msg.response.rmp.message = "Speech Setup request was satisfied: %s", msg.response.rmp.message.c_str();
//
//        // Add the request to the "setupSpeechActive_" list
//        temoto_2::getSpeech reqRes;
//        reqRes.request = req;
//        reqRes.response = res;
//        setupSpeechActive_.push_back(reqRes);
//
//        return true;
//    }
//    else
//    {
//        res.message = "Speech Setup request was not satisfied";
//        return true;
//    }
} 


void HumanContext::unloadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                  temoto_2::LoadSpeech::Response& res)
{
    std::string prefix = "[HumanContext::unloadSpeechCb]:";
	ROS_INFO("%s Unloading a speech or something.", prefix.c_str());
}

} // namespace human_context


/**
 * TODO: * Implement parsing multiple type requests: position(xyz) + orientation(rpy) + detected_gesture(str) + ...
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
//bool HumanContext::parseGestureSpecifier (temoto_2::gestureSpecifier spec)
//{

//}

/**
 * TODO: * Implement parsing multiple type requests
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
//bool HumanContext::parseSpeechSpecifier (temoto_2::speech_specifiers specs)
//{

//}
