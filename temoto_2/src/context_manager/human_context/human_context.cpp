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
HumanContext::HumanContext() : resource_manager_(srv_name::MANAGER, this)
{
  log_class_ = "";
  log_subsys_ = "human_context";
  log_group_ = "human_context";
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  
  // Fire up servers
  resource_manager_.addServer<temoto_2::LoadGesture>(
      srv_name::GESTURE_SERVER, &HumanContext::loadGestureCb, &HumanContext::unloadGestureCb);

  resource_manager_.addServer<temoto_2::LoadSpeech>(
      srv_name::SPEECH_SERVER, &HumanContext::loadSpeechCb, &HumanContext::unloadSpeechCb);

  TEMOTO_INFO("Human Context is ready.");
}

void HumanContext::loadGestureCb(temoto_2::LoadGesture::Request& req,
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

void HumanContext::unloadGestureCb(temoto_2::LoadGesture::Request& req,
                                   temoto_2::LoadGesture::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s Gesture unloaded.", prefix.c_str());
}

void HumanContext::loadSpeechCb(temoto_2::LoadSpeech::Request& req,
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

void HumanContext::unloadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                  temoto_2::LoadSpeech::Response& res)
{
  std::string prefix = "[HumanContext::unloadSpeechCb]:";
  TEMOTO_INFO("%s Speech unloaded.", prefix.c_str());
}

}  // namespace human_context

/**
 * TODO: * Implement parsing multiple type requests: position(xyz) + orientation(rpy) +
 * detected_gesture(str) + ...
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes
 * based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
// bool HumanContext::parseGestureSpecifier (temoto_2::gestureSpecifier spec)
//{

//}

/**
 * TODO: * Implement parsing multiple type requests
 *       * Add "filter" elements. If these make sense ... It might not be a good idea to build pipes
 * based on msgs
 *       * (?) Implement piping: raw_data -> filter_1 -> filter_n -> end_user
 */
