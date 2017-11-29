/*
 * TODO: * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */

#include "context_manager/context_manager.h"

namespace context_manager
{
ContextManager::ContextManager() : resource_manager_(srv_name::MANAGER, this)
{
  log_class_ = "";
  log_subsys_ = "context_manager";
  log_group_ = "context_manager";
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  
  /*
   * Start the servers
   */

  // Hand tracking service
  resource_manager_.addServer<temoto_2::LoadGesture>(srv_name::GESTURE_SERVER
                                                     , &ContextManager::loadGestureCb
                                                     , &ContextManager::unloadGestureCb);

  // Speech recognition service
  resource_manager_.addServer<temoto_2::LoadSpeech>(srv_name::SPEECH_SERVER
                                                    , &ContextManager::loadSpeechCb
                                                    , &ContextManager::unloadSpeechCb);

  TEMOTO_INFO("Context Manager is ready.");
}

void ContextManager::loadGestureCb(temoto_2::LoadGesture::Request& req,
                                 temoto_2::LoadGesture::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_INFO("%s Gesture requested.", prefix.c_str());
  TEMOTO_DEBUG("%s Using hardcoded specifiers[0]", prefix.c_str());

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.gesture_specifiers[0].type;

  // Call the sensor manager to arrange us a gesture sensor
  try
  {
    resource_manager_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                 sensor_manager::srv_name::SERVER, msg);
  }
  catch (...)
  {
    TEMOTO_ERROR("%s Service call failed.", prefix.c_str());
    return;  // TODO: throw here
  }

  TEMOTO_DEBUG("%s Got a response: '%s'", prefix.c_str(), msg.response.rmp.message.c_str());
  res.topic = msg.response.topic;
  res.package_name = msg.response.package_name;
  res.executable = msg.response.executable;
  res.rmp.code = msg.response.rmp.code;
  res.rmp.message = "Gesture request was ";
  res.rmp.message = +(msg.response.rmp.code == 0) ? "satisfied." : "not satisfied.";
  // TODO: send a reasonable response message and code
}

void ContextManager::unloadGestureCb(temoto_2::LoadGesture::Request& req,
                                   temoto_2::LoadGesture::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s Gesture unloaded.", prefix.c_str());
}

void ContextManager::loadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                temoto_2::LoadSpeech::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_INFO("%s Speech requested.", prefix.c_str());
  TEMOTO_DEBUG("%s Using hardcoded specifiers[0]", prefix.c_str());

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.speech_specifiers[0].type;

  // Call the sensor manager to arrange us a speech sensor
  try
  {
    resource_manager_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                 sensor_manager::srv_name::SERVER, msg);
  }
  catch (...)
  {
    TEMOTO_ERROR("%s Service call failed.", prefix.c_str());
    return;  // TODO: throw here
  }

  TEMOTO_DEBUG("%s Got a response: '%s'", prefix.c_str(), msg.response.rmp.message.c_str());
  res.topic = msg.response.topic;
  res.package_name = msg.response.package_name;
  res.executable = msg.response.executable;
  res.rmp.code = msg.response.rmp.code;
  res.rmp.message = "Speech request was ";
  res.rmp.message = +(msg.response.rmp.code == 0) ? "satisfied." : "not satisfied.";
  // TODO: send a reasonable response message and code
}

void ContextManager::unloadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                  temoto_2::LoadSpeech::Response& res)
{
  std::string prefix = "[ContextManager::unloadSpeechCb]:";
  TEMOTO_INFO("%s Speech unloaded.", prefix.c_str());
}

}  // namespace context_manager

/**
 * TODO: * Implement parsing multiple type requests: position(xyz) + orientation(rpy) +
 * detected_gesture(str) + ...
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes
 * based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
// bool ContextManager::parseGestureSpecifier (temoto_2::gestureSpecifier spec)
//{

//}

/**
 * TODO: * Implement parsing multiple type requests
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes
 * based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
