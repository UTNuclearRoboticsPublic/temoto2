/*
 * TODO: * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 *       * Decide wether its sensor manager's problem to look if different tasks
 *         are using the same resource or not. Its not a good idea to stop a node
 *         that is used by multiple tasks
 *       * START USING ROS NAMING CONVETIONS
 */

#include "context_manager/human_context/human_context.h"
#include "sensor_manager/sensor_manager_services.h"

HumanContext::HumanContext () : resource_manager_("temoto_2/human_context",this)
{
    // Start the clients
//    this->startSensorClient_ = n_.serviceClient<temoto_2::startSensorRequest>("start_sensor");
//    this->stopSensorClient_ = n_.serviceClient<temoto_2::stopSensorRequest>("stop_sensor");

    // Start the servers
    this->gestureServer_ = n_.advertiseService("setup_gesture", &HumanContext::setup_gesture_cb, this);
    this->speechServer_ = n_.advertiseService("setup_speech", &HumanContext::setup_speech_cb, this);
    this->stop_allocated_services_ = n_.advertiseService("stop_allocated_services_hc", &HumanContext::stopAllocatedServices, this);

    ROS_INFO("[HumanContext::HumanContext] all services and clients initialized, Human Context is good to go.");
}

/*
 * SETUP GESTURE
 */

bool HumanContext::setup_gesture_cb (temoto_2::getGestures::Request &req,
                                     temoto_2::getGestures::Response &res)
{
//    ROS_INFO("[HumanContext::setup_gestures_cb] Received a gesture setup request ...");
//
//    // Check the id of the req. If there is none (the first call from a task) then provide one
//    TemotoID::ID id_local = id_manager_.checkID(req.id);
//
//    // Look if similar resource is already allocated
//    for (auto& activeReq : setupGestureActive_)
//    {
//        if (compareGestureRequest(req, activeReq.request))
//        {
//            ROS_INFO("[HumanContext::setup_gestures_cb] Same request already available.");
//            res = activeReq.response;
//            res.id = static_cast<int>(id_local);
//            return true;
//        }
//    }
//
//    // The request was not in "setupSpeechActive_" list. Make a new sensor request
//    ROS_INFO("[HumanContext::setup_gestures_cb] This request is unique. Setting up the sensor ...");
//
//    temoto_2::LoadSensor msg;
//    msg.request.sensor_type = req.gesture_specifiers[0].type;
//
//    // Call the sensor manager to arrange us a gesture sensor
//    try
//    {
//        resource_manager_.call<temoto_2::LoadSensor>(
//                sensor_manager::srv_name::MANAGER,
//                sensor_manager::srv_name::SERVER,
//                msg);
//    }
//    catch(...)
//    {
//        ROS_ERROR("[HumanContext::setup_gestures_cb] Failed to call service /start_sensor, trying again...");
//    }
//
//    ROS_INFO("[HumanContext::setup_gestures_cb] Got a response from service /start_sensor: '%s'", msg.response.rmp.message.c_str());
//    res.id = static_cast<int>(id_local);
//    res.topic = msg.response.topic;
//    res.name = msg.response.package_name;
//    res.executable = msg.response.executable;
//    res.code = msg.response.rmp.code;
//
//    // Check the response message, if all is ok then
//    if (msg.response.rmp.code == 0)
//    {
//        res.message = msg.response.rmp.message = "Gesture Setup request was satisfied: %s", msg.response.rmp.message.c_str();
//
//        // Add the request to the "setupSpeechActive_" list
//        temoto_2::getGestures reqRes;
//        reqRes.request = req;
//        reqRes.response = res;
//        setupGestureActive_.push_back(reqRes);
//
//        return true;
//    }
//    else
//    {
//        res.message = "Gesture Setup request was not satisfied";
//        return true;
//    }
}

/*
 * TODO: * Pass informative response messages
 *
 */
bool HumanContext::setup_speech_cb (temoto_2::getSpeech::Request &req,
                                    temoto_2::getSpeech::Response &res)
{
    ROS_INFO("[HumanContext::setup_speech_cb] Received a speech setup request with specifiers[0]: '%s', '%s', '%s'",
			req.speech_specifiers[0].type.c_str(), req.speech_specifiers[0].package.c_str(), req.speech_specifiers[0].executable.c_str());

    // Check the id of the req. If there is none (the first call from a task) then provide one
    TemotoID::ID id_local = id_manager_.checkID(req.id);

    for (auto& activeReq : setupSpeechActive_)
    {
        if (compareSpeechRequest(req, activeReq.request))
        {
            ROS_INFO("[HumanContext::setup_speech_cb] Same request already available.");
            res = activeReq.response;
            res.id = static_cast<int>(id_local);
            return true;
        }
    }

    // The request was not in "setupSpeechActive_" list. Make a new sensor request
    ROS_INFO("[HumanContext::setup_speech_cb] This request is unique. Setting up the sensor ...");

    temoto_2::LoadSensor msg;
    msg.request.sensor_type = req.speech_specifiers[0].type;

    // Call the sensor manager
    try
    {
        resource_manager_.call<temoto_2::LoadSensor>(
                sensor_manager::srv_name::MANAGER,
                sensor_manager::srv_name::SERVER,
                msg);
    }
    catch(...)
    {
        ROS_ERROR("[HumanContext::setup_speech_cb] Failed to call service /start_sensor, trying again...");
    }

    ROS_INFO("[HumanContext::setup_speech_cb] Got a response from service /start_sensor: '%s'", msg.response.rmp.message.c_str());
    res.id = static_cast<int>(id_local);
    res.topic = msg.response.topic;
    res.name = msg.response.package_name;
    res.executable = msg.response.executable;
    res.code = msg.response.rmp.code;

    // Check the response message, if all is ok then
    if (msg.response.rmp.code == 0)
    {   
        res.message = msg.response.rmp.message = "Speech Setup request was satisfied: %s", msg.response.rmp.message.c_str();

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
        return true;
    }
}


/*
 * TODO: send a reasonable response message and code
 */

bool HumanContext::stopAllocatedServices (temoto_2::stopAllocatedServices::Request& req,
                                          temoto_2::stopAllocatedServices::Response& res)
{
	// TODO: implement in RMP
    ROS_INFO("[HumanContext::stopAllocatedServices] Received a 'stopAllocatedServices' request to ID: '%ld'.", req.id);

    // Default the response code to 0
    res.code = 0;

    // Go through the "setupGestureActive_" and "setupSpeechActive_" lists, look for the id
    // and make a request to the Sensor Manager to stop those services.

//    for (auto it = setupSpeechActive_.begin(); it != setupSpeechActive_.end(); /*empty*/)
//    {
//        if( req.id == (*it).response.id )
//        {
//            temoto_2::UnloadResource msg;
//            msg.request.name = (*it).response.name;
//            msg.request.executable = (*it).response.executable;
//
//            // Call the service
////            while (!stopSensorClient_.call(stop_sens_local))
////            {
////                ROS_ERROR("[HumanContext::stopAllocatedServices] Failed to call service /stop_sensor, trying again...");
////            }
//
//            ROS_INFO("[HumanContext::stopAllocatedServices] /stop_sensor responded: %s", msg.response.message.c_str());
//
//            if (msg.response.code == 0)
//            {
//                res.code = msg.response.code;
//                setupSpeechActive_.erase (it);
//            }
//        }
//        else
//            it++;
//    }
//
//    for (auto it = setupGestureActive_.begin(); it != setupGestureActive_.end(); /*empty*/)
//    {
//        if( req.id == (*it).response.id )
//        {
//            temoto_2::stopSensorRequest stop_sens_local;
//            stop_sens_local.request.name = (*it).response.name;
//            stop_sens_local.request.executable = (*it).response.executable;
//
//            // Call the service
////            while (!stopSensorClient_.call(stop_sens_local))
////            {
////                ROS_ERROR("[HumanContext::stopAllocatedServices] Failed to call service /stop_sensor, trying again...");
////            }
//
//            ROS_INFO("[HumanContext::stopAllocatedServices] /stop_sensor responded: %s", stop_sens_local.response.message.c_str());
//
//            if (stop_sens_local.response.code == 0)
//            {
//                res.code = stop_sens_local.response.code;
//                setupGestureActive_.erase (it);
//            }
//        }
//        else
//            it++;
//    }
//
    return true;
}


/*
 * TODO: * Implement a complete comparison
 */
bool HumanContext::compareGestureRequest (temoto_2::getGestures::Request &req,
                                          temoto_2::getGestures::Request &reqLocal) const
{
    std::vector <temoto_2::gestureSpecifier> specifiersLoc = reqLocal.gesture_specifiers;

    // first check if the devices match
    if (specifiersLoc[0].dev.compare(req.gesture_specifiers[0].dev) == 0)
    {
        return true;
    }

    return false;
}


/*
 * TODO: * Implement a complete comparison
 */
bool HumanContext::compareSpeechRequest (temoto_2::getSpeech::Request &req,
                                         temoto_2::getSpeech::Request &reqLocal) const
{
    std::vector <temoto_2::speechSpecifier> specifiersLoc = reqLocal.speech_specifiers;

    // first check if the devices match
    if (specifiersLoc[0].dev.compare(req.speech_specifiers[0].dev) == 0)
    {
        return true;
    }

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
//bool HumanContext::parseSpeechSpecifier (temoto_2::speech_specifiers specs)
//{

//}
