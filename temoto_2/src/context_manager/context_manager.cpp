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

  /*
   * Process the tracking methods that are described in an external YAML file
   */

  // Path to the trackers YAML file
  std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                                "tracking_methods.yaml";

  // Parse the trackes
  parseTrackers(yaml_filename);

  // Print out the trackers
  for (auto& tracker_category : categorized_trackers_)
  {
    std::cout << "CATEGORY: " << tracker_category.first << std::endl;
    for (auto& tracking_method : tracker_category.second)
    {
      std::cout << tracking_method.toString() << std::endl;
    }
  }
  
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
  TEMOTO_INFO_STREAM(prefix << " Received a request: \n" << req << std::endl);

  try
  {
    // Get the tracking methods of the requested category
    auto trackers = categorized_trackers_.find(req.tracker_category);
    std::cout << "D0 \n";

    // Proceed if the requested tracker category exists
    if (trackers != categorized_trackers_.end())
    {
      std::cout << "D1 \n";
      // Choose a tracker based on a TODO metric
      const TrackerInfo& tracker = trackers->second.at(0);

      std::cout << "D2 \n";
      // Create a unique pipe identifier string
      std::string pipeID = "pipe_" + std::to_string(pipeIDGenerator.generateID())
                         + "_at_" + common::getTemotoNamespace();

      /*
       * Build the pipe based on the number of filters. If the pipe
       * contains only one filter, then there are no constraints on
       * the ouptut topic types. But if the pipe contains multiple filters
       * then each preceding filter has to provide the topics that are
       * required by the proceding filter
       */
      TopicContainer required_topics;

      if (tracker.getPipeSize() > 1)
      {
        // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
        // for-loop, then it crashes on the second loop. Not sure why.
        std::set<std::string> req_tops = tracker.getPipe().at(1).required_input_topic_types_;

        // Loop over requested topics
        for (auto& topic : req_tops)
        {
          required_topics.addOutputTopicType(topic);
        }
      }

      // Loop over the pipe
      std::vector<Filter> pipe = tracker.getPipe();

      for (unsigned int i=0; i<pipe.size(); i++)
      {
        /*
         * If the filter is a sensor
         */
        if (pipe.at(i).filter_category_ == "sensor")
        {
          // Compose the LoadSensor message
          temoto_2::LoadSensor load_sensor_msg;
          load_sensor_msg.request.sensor_type = pipe.at(i).filter_type_;
          load_sensor_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

          // Call the Sensor Manager
          resource_manager_2_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                         sensor_manager::srv_name::SERVER,
                                                         load_sensor_msg);

          required_topics.setInputTopicsByKeyValue(load_sensor_msg.response.output_topics);
        }

        /*
         * If the filter is an algorithm
         */
        else if (pipe.at(i).filter_category_ == "algorithm")
        {

          // Clear out the required output topics
          required_topics.clearOutputTopics();

          // If it is not the last filter then ...
          if (i != pipe.size()-1)
          {
            // ... get the requirements for the output topics from the proceding filter
            for (auto& topic : pipe.at(i+1).required_input_topic_types_)
            {
              required_topics.addOutputTopic(topic, "/" + pipeID + "/filter_" + std::to_string(i) + "/" + topic);
            }
          }
          else
          {
            // ... get the requirements for the output topics from own output topic requirements
            // TODO: throw if the "required_output_topic_types_" is empty
            for (auto& topic : pipe.at(i).required_output_topic_types_)
            {
              required_topics.addOutputTopic(topic, "/" + pipeID + "/filter_" + std::to_string(i) + "/" + topic);
            }
          }

          // Compose the LoadAlgorithm message
          temoto_2::LoadAlgorithm load_algorithm_msg;
          load_algorithm_msg.request.algorithm_type = pipe.at(i).filter_type_;
          load_algorithm_msg.request.input_topics = required_topics.inputTopicsAsKeyValues();
          load_algorithm_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

          // Call the Algorithm Manager
          resource_manager_2_.call<temoto_2::LoadAlgorithm>(algorithm_manager::srv_name::MANAGER,
                                                            algorithm_manager::srv_name::SERVER,
                                                            load_algorithm_msg);

          required_topics.setInputTopicsByKeyValue(load_algorithm_msg.response.output_topics);
        }
      }

      // Send the output topics of the last filter back via response
      res.output_topics = required_topics.outputTopicsAsKeyValues();
    }
    else
    {
      res.rmp.errorStack = error_handler_.createAndReturn(999, prefix, "No trackers found for the requested category");
    }

  // Catch the errors
  }
  catch(error::ErrorStack& e)
  {
    res.rmp.errorStack = error_handler_.forwardAndReturn(e, prefix);
  }
  catch(...)
  {
    res.rmp.errorStack = error_handler_.createAndReturn(999, prefix, "Unhandled exception");
  }

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

/*
 * Parse trackers
 */
void ContextManager::parseTrackers(std::string config_path)
{
  // Read in the config file
  std::ifstream in(config_path);
  YAML::Node config = YAML::Load(in);

  // Check if it is a map
  if (!config.IsMap())
  {
    // TODO Throw
    std::cout << " throw throw throw \n";
    return;
  }

  // Iterate over different tracker categories (hand trackers, artag trackers, ...)
  for (YAML::const_iterator tracker_type_it = config.begin(); tracker_type_it != config.end(); ++tracker_type_it)
  {
    // Each category must contain a sequence of tracking methods
    if (!tracker_type_it->second.IsSequence())
    {
      // TODO Throw
      std::cout << " throw TODO throw TODO \n";
      return;
    }

    // Get the category of the tracker
    std::string tracker_category = tracker_type_it->first.as<std::string>();

    // Iterate over different tracking methods within the given category
    for (YAML::const_iterator method_it = tracker_type_it->second.begin();
         method_it != tracker_type_it->second.end();
         ++method_it)
    {
      try
      {
        // Convert the tracking method yaml description into TrackerInfo
        context_manager::TrackerInfo tracker_info = method_it->as<context_manager::TrackerInfo>();

        // Add the tracking method into the map of locally known trackers
        categorized_trackers_[tracker_category].push_back(tracker_info);

        // TODO: Print via TEMOTO_DEBUG
        // std::cout << tracker_info.toString() << std::endl;
      }
      catch (YAML::InvalidNode e)
      {
        // print out the error message
        std::cout << "Conversion failed: " << e.what() << std::endl;
      }
    }
  }
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
