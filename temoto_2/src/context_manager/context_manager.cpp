#include "context_manager/context_manager.h"
#include "ros/package.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace context_manager
{
ContextManager::ContextManager()
  : resource_manager_1_(srv_name::MANAGER, this)
  , resource_manager_2_(srv_name::MANAGER_2, this)
  , object_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &ContextManager::objectSyncCb, this)
{
  class_name_ = __func__;
  subsystem_name_ = "context_manager";
  subsystem_code_ = error::Subsystem::CONTEXT_MANAGER;
  log_group_ = "context_manager";
  error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);
  
  /*
   * Start the servers
   */

  // Hand tracking service
  resource_manager_1_.addServer<temoto_2::LoadGesture>(srv_name::GESTURE_SERVER
                                                     , &ContextManager::loadGestureCb
                                                     , &ContextManager::unloadGestureCb);

  // Speech recognition service
  resource_manager_1_.addServer<temoto_2::LoadSpeech>(srv_name::SPEECH_SERVER
                                                    , &ContextManager::loadSpeechCb
                                                    , &ContextManager::unloadSpeechCb);

  // Speech recognition service
  resource_manager_2_.addServer<temoto_2::LoadTracker>(srv_name::TRACKER_SERVER
                                                    , &ContextManager::loadTrackerCb
                                                    , &ContextManager::unloadTrackerCb);


  // "Add object" server
  add_objects_server_ = nh_.advertiseService(srv_name::SERVER_ADD_OBJECTS, &ContextManager::addObjectsCb, this);
  
  // Request remote objects
  object_syncer_.requestRemoteConfigs();
  
  TEMOTO_INFO("Context Manager is ready.");
}

/*
 * Object synchronization callback
 */
void ContextManager::objectSyncCb(const temoto_2::ConfigSync& msg, const Objects& payload)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseAllObjects();
    return;
  }

  // Add or update objects
  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG("%s Received a payload", prefix.c_str());
    addOrUpdateObjects(payload, true);
  }
}

/*
 * Object update callback
 */
void ContextManager::addOrUpdateObjects(const Objects& objects_to_add, bool from_other_manager)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  for (auto& object : objects_to_add)
  {
    // Check if the object has to be added or updated
    auto it = std::find_if(objects_.begin(), objects_.end(),
        [&](const ObjectPtr& o_ptr) { return *o_ptr == object; });

    // Update the object
    if (it != objects_.end())
    {
      TEMOTO_DEBUG("%s Updating object: '%s'", prefix.c_str(), object.name.c_str());
      *it = std::make_shared<temoto_2::ObjectContainer>(object);
    }

    // Add new object
    else
    {
      TEMOTO_DEBUG("%s Adding new object: '%s'", prefix.c_str(), object.name.c_str());
      objects_.push_back(std::make_shared<temoto_2::ObjectContainer>(object));
    }
  }

  // If this object was added by own namespace, then advertise this config to other managers
  if (!from_other_manager)
  {
    TEMOTO_DEBUG("%s Advertising the objects to other namespaces", prefix.c_str());
    object_syncer_.advertise(objects_to_add);
  }
}

/*
 * Advertise all objects
 */
void ContextManager::advertiseAllObjects()
{
  // Publish all objects
  Objects objects_payload;

  for(auto& object : objects_)
  {
    objects_payload.push_back(*object);
  }

  // Send to other managers if there is anything to send
  if(objects_payload.size())
  {
    object_syncer_.advertise(objects_payload);
  }
}

/*
 * Callback for adding objects
 */
bool ContextManager::addObjectsCb(temoto_2::AddObjects::Request& req, temoto_2::AddObjects::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG_STREAM(prefix << "received a request to add %d objects: \n" << req.objects.size());

  addOrUpdateObjects(req.objects, false);

  return true;
}

/*
 * Load tracker callback
 */
void ContextManager::loadTrackerCb(temoto_2::LoadTracker::Request& req,
                                   temoto_2::LoadTracker::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_INFO_STREAM(prefix << "%s Received a request: \n" << req << std::endl);

//  try
//  {
//    // Load a calibrated camera
//    temoto_2::LoadSensor load_sensor_msg;


//    resource_manager_2_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
//                                                   sensor_manager::srv_name::SERVER,
//                                                   load_sensor_msg);
//  }
//  catch(error::ErrorStack& e)
//  {
//    res.rmp.errorStack = error_handler_.forwardAndReturn(e, prefix);
//  }
//  catch(...)
//  {
//    res.rmp.errorStack = error_handler_.createAndReturn(999, prefix, "Unhandled exception");
//  }
}

/*
 * Unload tracker callback
 */
void ContextManager::unloadTrackerCb(temoto_2::LoadTracker::Request& req, temoto_2::LoadTracker::Response& res)
{
  // POOLELI
}

/*
 * Load gesture callback
 */
void ContextManager::loadGestureCb(temoto_2::LoadGesture::Request& req,
                                   temoto_2::LoadGesture::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_INFO("%s Gesture requested.", prefix.c_str());
  TEMOTO_DEBUG("%s Using hardcoded specifiers[0]", prefix.c_str());

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.gesture_specifiers[0].type;

  // Call the sensor manager to arrange us a gesture sensor
  try
  {
    resource_manager_1_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                   sensor_manager::srv_name::SERVER,
                                                   msg);
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
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s Gesture unloaded.", prefix.c_str());
}

void ContextManager::loadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                  temoto_2::LoadSpeech::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_INFO("%s Speech requested.", prefix.c_str());
  TEMOTO_DEBUG("%s Using hardcoded specifiers[0]", prefix.c_str());

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.speech_specifiers[0].type;

  // Call the sensor manager to arrange us a speech sensor
  try
  {
    resource_manager_1_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
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
