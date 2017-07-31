/*
 * TODO: * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */

#include "context_manager/human_context/human_context.h"

HumanContext::HumanContext ()
{
    // Start the clients
    this->startSensorClient_ = n_.serviceClient<temoto_2::startSensorRequest>("start_sensor");
    this->stopSensorClient_ = n_.serviceClient<temoto_2::stopSensorRequest>("stop_sensor");

    // Start the servers
    this->gestureServer_ = n_.advertiseService("setup_gesture", &HumanContext::setup_gesture_cb, this);
    this->speechServer_ = n_.advertiseService("setup_speech", &HumanContext::setup_speech_cb, this);

    ROS_INFO("[HumanContext::HumanContext] all services and clients initialized, Human Context is good to go.");
}

bool HumanContext::setup_gesture_cb (temoto_2::getGestures::Request &req,
                                     temoto_2::getGestures::Response &res)
{
    return true;
}

/*
 * TODO: * Pass informative response messages
 *
 */
bool HumanContext::setup_speech_cb (temoto_2::getSpeech::Request &req,
                                    temoto_2::getSpeech::Response &res)
{
    ROS_INFO("[HumanContext::setup_speech_cb] Received a speech setup request ...");

    for (auto& activeReq : setupSpeechActive_)
    {
        if (compareSpeechRequest(req, activeReq.request))
        {
            ROS_INFO("[HumanContext::setup_speech_cb] Same request already available.");
            res = activeReq.response;
            return true;
        }

    }

    // The request was not in "setupSpeechActive_" list. Make a new sensor request
    ROS_INFO("[HumanContext::setup_speech_cb] This request is unique. Setting up the sensor ...");

    temoto_2::startSensorRequest startSensReq;
    startSensReq.request.type = req.speechSpecifiers[0].type;

    // Call the server
    while (!startSensorClient_.call(startSensReq))
    {
        ROS_ERROR("[HumanContext::setup_speech_cb] Failed to call service /start_sensor, trying again...");
    }

    ROS_INFO("[HumanContext::setup_speech_cb] Got a response from service /start_sensor: '%s'", startSensReq.response.message.c_str());
    res.topic = startSensReq.response.topic;
    res.code = startSensReq.response.code;

    if (startSensReq.response.code == 0)
    {
        res.message = startSensReq.response.message = "Speech Setup request was satisfied";

        // Add the request to the "setupSpeechActive_" list
        temoto_2::getSpeech reqRes;
        reqRes.request = req;
        reqRes.response = res;
        setupSpeechActive_.push_back(reqRes);

        return true;
    }
    else
    {
        res.message = "Speech Setup request was not satisfied";
        return false;
    }
}



/*
 * TODO: * Implement a complete comparison
 */
bool HumanContext::compareSpeechRequest (temoto_2::getSpeech::Request &req,
                                         temoto_2::getSpeech::Request &reqLocal) const
{
    std::vector <temoto_2::speechSpecifier> specifiersLoc = reqLocal.speechSpecifiers;

    // first check if the devices match
    if (specifiersLoc[0].dev.compare(req.speechSpecifiers[0].dev) == 0)
    {
        return true;
    }
    /*
    for(auto& )

    if (reqLocal.type.compare(req.type) == 0)
    {
     return true;
    }
    */

    return false;
}


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
//bool HumanContext::parseSpeechSpecifier (temoto_2::speechSpecifiers specs)
//{

//}
