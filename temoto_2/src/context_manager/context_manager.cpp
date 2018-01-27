#include "context_manager/context_manager.h"
#include "ros/package.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace context_manager
{

ContextManager::ContextManager()
  : BaseSubsystem("context_manager", error::Subsystem::CONTEXT_MANAGER, __func__)
  , resource_manager_1_(srv_name::MANAGER, this)
  , resource_manager_2_(srv_name::MANAGER_2, this)
  , tracked_objects_syncer_(srv_name::MANAGER, srv_name::SYNC_TRACKED_OBJECTS_TOPIC, &ContextManager::trackedObjectsSyncCb, this)
  , object_syncer_(srv_name::MANAGER, srv_name::SYNC_OBJECTS_TOPIC, &ContextManager::objectSyncCb, this)
  , tracker_core_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/../object_trackers")
{
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

  // Object tracking service
  resource_manager_1_.addServer<temoto_2::TrackObject>(srv_name::TRACK_OBJECT_SERVER
                                                    , &ContextManager::loadTrackObjectCb
                                                    , &ContextManager::unloadTrackObjectCb);

  // Tracker setup service
  resource_manager_2_.addServer<temoto_2::LoadTracker>(srv_name::TRACKER_SERVER
                                                    , &ContextManager::loadTrackerCb
                                                    , &ContextManager::unloadTrackerCb);

  // Register callback for status info
  resource_manager_1_.registerStatusCb(&ContextManager::statusCb1);
  resource_manager_2_.registerStatusCb(&ContextManager::statusCb2);

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
//  for (auto& tracker_category : categorized_trackers_)
//  {
//    std::cout << "CATEGORY: " << tracker_category.first << std::endl;
//    for (auto& tracking_method : tracker_category.second)
//    {
//      std::cout << tracking_method.toString() << std::endl;
//    }
//  }
  
  TEMOTO_INFO("Context Manager is ready.");
}

/*
 * Object synchronization callback
 */
void ContextManager::objectSyncCb(const temoto_2::ConfigSync& msg, const Objects& payload)
{
  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseAllObjects();
    return;
  }

  // Add or update objects
  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG("Received a payload.");
    addOrUpdateObjects(payload, true);
  }
}

/*
 * Tracked objects synchronization callback
 */
void ContextManager::trackedObjectsSyncCb(const temoto_2::ConfigSync& msg, const std::string& payload)
{
  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG_STREAM("Received a message, that '" << payload << "' is tracked by '"
                        << msg.temoto_namespace << "'.");

    // Add a notion about a object that is being tracked
    m_tracked_objects_remote_[payload] = msg.temoto_namespace;
  }
  else
  if (msg.action == rmp::sync_action::REMOVE_CONFIG)
  {
    TEMOTO_DEBUG_STREAM("Received a message, that '" << payload << "' is not tracked by '"
                        << msg.temoto_namespace << "' anymore.");

    // Remove a notion about a object that is being tracked
    m_tracked_objects_remote_.erase(payload);
  }
}

/*
 * Object update callback
 */
void ContextManager::addOrUpdateObjects(const Objects& objects_to_add, bool from_other_manager)
{
  // Loop over the list of provided objects
  for (auto object : objects_to_add)
  {
    // Replace all spaces in the name with the underscore character
    std::replace(object.name.begin(), object.name.end(), ' ', '_');

    // Check if the object has to be added or updated
    auto it = std::find_if(objects_.begin(), objects_.end(),
        [&](const ObjectPtr& o_ptr) { return *o_ptr == object; });

    // Update the object
    if (it != objects_.end())
    {
      TEMOTO_DEBUG("Updating object: '%s'.", object.name.c_str());
      *it = std::make_shared<temoto_2::ObjectContainer>(object);
    }

    // Add new object
    else
    {
      TEMOTO_DEBUG("Adding new object: '%s'.", object.name.c_str());
      objects_.push_back(std::make_shared<temoto_2::ObjectContainer>(object));
    }
  }

  // If this object was added by its own namespace, then advertise this config to other managers
  if (!from_other_manager)
  {
    TEMOTO_DEBUG("Advertising the objects to other namespaces.");
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

  for (auto& object : objects_)
  {
    objects_payload.push_back(*object);
  }

  // Send to other managers if there is anything to send
  if (objects_payload.size())
  {
    object_syncer_.advertise(objects_payload);
  }
}

/*
 * Find object
 */
ObjectPtr ContextManager::findObject(std::string object_name)
{
  for (auto& object : objects_)
  {
    if (object->name == object_name)
    {
      return object;
    }
  }

  // Throw an error if no objects were found
  throw CREATE_ERROR(error::Code::UNKNOWN_OBJECT, "The requested object is unknown");
}


/*
 * Callback for adding objects
 */
bool ContextManager::addObjectsCb(temoto_2::AddObjects::Request& req, temoto_2::AddObjects::Response& res)
{
  TEMOTO_DEBUG("Received a request to add %ld objects.", req.objects.size());

  addOrUpdateObjects(req.objects, false);

  return true;
}

/*
 * Server for tracking objects
 */
void ContextManager::loadTrackObjectCb(temoto_2::TrackObject::Request& req, temoto_2::TrackObject::Response& res)
{
  try
  {
    // Replace all spaces in the name with the underscore character
    std::string object_name_no_space = req.object_name;
    std::replace(object_name_no_space.begin(), object_name_no_space.end(), ' ', '_');

    TEMOTO_DEBUG_STREAM("Received a request to track an object named: '" << object_name_no_space << "'");

    /*
     * Check if this object is already tracked in other temoto namespace
     */
    std::string remote_temoto_namespace = m_tracked_objects_remote_[object_name_no_space];

    if (!remote_temoto_namespace.empty())
    {
      TEMOTO_DEBUG_STREAM("The object '" << object_name_no_space << "' is alerady tracked by '"
                          << remote_temoto_namespace << "'. Forwarding the request.");

      temoto_2::TrackObject track_object_msg;
      track_object_msg.request = req;

      // Send the request to the remote namespace
      resource_manager_2_.template call<temoto_2::TrackObject>(context_manager::srv_name::MANAGER,
                                                               context_manager::srv_name::TRACK_OBJECT_SERVER,
                                                               track_object_msg,
                                                               rmp::FailureBehavior::NONE,
                                                               remote_temoto_namespace);

      res = track_object_msg.response;
      return;
    }

    // Look if the requested object is described in the object database
    ObjectPtr requested_object = findObject(object_name_no_space);

    TEMOTO_DEBUG_STREAM("The requested object is known, tracking it via: " << requested_object->detection_methods[0]);

    /*
     * Start a tracker that can be used to detect the requested object
     */
    temoto_2::LoadTracker load_tracker_msg;
    auto& detection_methods = requested_object->detection_methods;
    std::string selected_tracker;

    // Loop over different tracker categories and try to load one. The loop is iterated either until
    // a tracker is succesfully loaded or options are exhausted (failure)
    for (auto& tracker_category : detection_methods)
    {
      try
      {
        load_tracker_msg = temoto_2::LoadTracker();
        load_tracker_msg.request.tracker_category = tracker_category;
        resource_manager_1_.call<temoto_2::LoadTracker>(context_manager::srv_name::MANAGER_2,
                                                        context_manager::srv_name::TRACKER_SERVER,
                                                        load_tracker_msg);
      }
      catch (error::ErrorStack& error_stack)
      {
        // If a requested tracker was not found but there are other options
        // available, then continue. Otherwise forward the error
        if (error_stack.front().code == static_cast<int>(error::Code::NO_TRACKERS_FOUND) &&
            &tracker_category != &detection_methods.back())
        {
          continue;
        }

        throw FORWARD_ERROR(error_stack);
      }

      selected_tracker = tracker_category;
    }

    /*
     * Start the specific object tracker. Since there are different general object
     * tracking methods and each tracker outputs different types of data, then
     * the specific tracking has to be set up based on the general tracker. For example
     * a general tracker, e.g. AR tag detector, publshes data about detected tags. The
     * specific tracker has to subscribe to the detected tags topic and since the
     * tags are differentiated by the tag ID, the specific tracker has to know the ID
     * beforehand.
     */

    /*
     * Get the topic where the tracker publishes its output data
     */
    TopicContainer tracker_topics;
    tracker_topics.setOutputTopicsByKeyValue(load_tracker_msg.response.output_topics);

    // Topic where the information about the required object is going to be published.
    std::string tracked_object_topic = common::getAbsolutePath("object_tracker/" + object_name_no_space);

    /*
     * AR tag based object tracker setup
     */
    if (selected_tracker == "artags")
    {
      TEMOTO_DEBUG_STREAM("Using AR-tag based tracking");

      // Get the AR tag data dopic
      std::string tracker_data_topic = tracker_topics.getOutputTopic("marker_data");

      /*
       * TTP related stuff up ahead: A semantic frame is manually created. Based on that SF
       * a SF tree is created, given that an action implementation, that corresponds to the
       * manually created SF, exists. The specific tracker task is started and it continues
       * running in the background until its ordered to be stopped.
       */

      std::string action = "track";
      TTP::Subjects subjects;

      // Subject that will contain the name of the tracked object.
      // Necessary when the tracker has to be stopped
      TTP::Subject sub_0("what", object_name_no_space);

      // Subject that will contain the data necessary for the specific tracker
      TTP::Subject sub_1("what", "artag data");

      // Topic from where the raw AR tag tracker data comes from
      sub_1.addData("topic", tracker_data_topic);

      // Topic where the AImp must publish the data about the tracked object
      sub_1.addData("topic", tracked_object_topic);

      // This object will be updated inside the tracking AImp (action implementation)
      sub_1.addData("pointer", boost::any_cast<ObjectPtr>(requested_object));

      subjects.push_back(sub_0);
      subjects.push_back(sub_1);

      // Create a SF
      std::vector<TTP::TaskDescriptor> task_descriptors;
      task_descriptors.emplace_back(action, subjects);
      task_descriptors[0].setActionStemmed(action);

      // Create a sematic frame tree
      TTP::TaskTree sft = TTP::SFTBuilder::build(task_descriptors);

      // Get the root node of the tree
      TTP::TaskTreeNode& root_node = sft.getRootNode();
      sft.printTaskDescriptors(root_node);

      // Execute the SFT
      tracker_core_.executeSFT(std::move(sft));

      // Put the object into the list of tracked objects. This is used later
      // for stopping the tracker
      m_tracked_objects_local_[res.rmp.resource_id] = object_name_no_space;
    }

    /*
     * Hand based object tracker setup
     */
    if (selected_tracker == "hands")
    {
      TEMOTO_DEBUG_STREAM("Using hand based tracking");

      // Get the AR tag data dopic
      std::string tracker_data_topic = tracker_topics.getOutputTopic("handtracker_data");

      /*
       * TTP related stuff up ahead: A semantic frame is manually created. Based on that SF
       * a SF tree is created, given that an action implementation, that corresponds to the
       * manually created SF, exists. The specific tracker task is started and it continues
       * running in the background until its ordered to be stopped.
       */

      std::string action = "track";
      TTP::Subjects subjects;

      // Subject that will contain the name of the tracked object.
      // Necessary when the tracker has to be stopped
      TTP::Subject sub_0("what", object_name_no_space);

      // Subject that will contain the data necessary for the specific tracker
      TTP::Subject sub_1("what", "hand data");

      // Topic from where the raw hand tracker data comes from
      sub_1.addData("topic", tracker_data_topic);

      // Topic where the AImp must publish the data about the tracked object
      sub_1.addData("topic", tracked_object_topic);

      // This object will be updated inside the tracking AImp (action implementation)
      sub_1.addData("pointer", boost::any_cast<ObjectPtr>(requested_object));

      subjects.push_back(sub_0);
      subjects.push_back(sub_1);

      // Create a SF
      std::vector<TTP::TaskDescriptor> task_descriptors;
      task_descriptors.emplace_back(action, subjects);
      task_descriptors[0].setActionStemmed(action);

      // Create a sematic frame tree
      TTP::TaskTree sft = TTP::SFTBuilder::build(task_descriptors);

      // Get the root node of the tree
      TTP::TaskTreeNode& root_node = sft.getRootNode();
      sft.printTaskDescriptors(root_node);

      // Execute the SFT
      tracker_core_.executeSFT(std::move(sft));

      // Put the object into the list of tracked objects. This is used later
      // for stopping the tracker
      m_tracked_objects_local_[res.rmp.resource_id] = object_name_no_space;
    }

    res.object_topic = tracked_object_topic;

    // Let context managers in other namespaces know, that this object is being tracked
    tracked_objects_syncer_.advertise(object_name_no_space);

  }
  catch (error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

/*
 * Unload the track object
 */
void ContextManager::unloadTrackObjectCb(temoto_2::TrackObject::Request& req,
                                         temoto_2::TrackObject::Response& res)
{
  /*
   * Stopping tracking the object based on its name
   */
  try
  {
    // Check if the object is tracked locally or by a remote manager
    if (!m_tracked_objects_remote_[req.object_name].empty())
    {
      // The object is tracked by a remote manager
      return;
    }

    // Get the name of the tracked object
    std::string tracked_object = m_tracked_objects_local_[res.rmp.resource_id];

    if (tracked_object.empty())
    {
      throw CREATE_ERROR(error::Code::NO_TRACKERS_FOUND, std::string("The object '") +
                         req.object_name + "' is not tracked");
    }

    TEMOTO_DEBUG_STREAM("Received a request to stop tracking an object named: '"
                        << tracked_object << "'");

    // Stop tracking the object
    tracker_core_.stopTask("", tracked_object);

    // Erase the object from the map of tracked objects
    m_tracked_objects_local_.erase(res.rmp.resource_id);

    // Let context managers in other namespaces know, that this object is not tracked anymore
    tracked_objects_syncer_.advertise(tracked_object, rmp::sync_action::REMOVE_CONFIG);
  }
  catch (error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

/**
 * @brief findTrackers
 * @param req
 * @return
 */
TrackerInfoPtrs ContextManager::findTrackers(temoto_2::LoadTracker::Request& req)
{
  // Get the tracking methods of the requested category
  auto tracker_category = categorized_trackers_.find(req.tracker_category);

  // Throw an error if the requested tracker category does not exist
  if (tracker_category == categorized_trackers_.end())
  {
    throw CREATE_ERROR(error::Code::NO_TRACKERS_FOUND, "No trackers found for the requested category");
  }

  // Get the trackers
  TrackerInfoPtrs trackers = tracker_category->second;

  // Check if there are any required types for the output topics of the tracker
  if (!req.output_topics.empty())
  {
    // Loop over trackers
    for (auto tracker_it = trackers.begin(); tracker_it != trackers.end(); /* empty */)
    {
      bool tracker_suitable = true;

      // Create a copy of the required topics
      std::vector<diagnostic_msgs::KeyValue> req_topic_types = req.output_topics;
      std::set<std::string> last_filter_topics = (*tracker_it)->getPipe().back().required_output_topic_types_;

      // The topics that the last filter of the pipe provides
      for (auto& last_filter_topic : last_filter_topics)
      {
        bool topic_found = false;

        // Compare the topic of the last filter with the required topics
        for (auto rtt_it = req_topic_types.begin(); rtt_it != req_topic_types.end(); rtt_it++)
        {
          if (rtt_it->key == last_filter_topic)
          {
            topic_found = true;
            req_topic_types.erase(rtt_it);
            break;
          }
        }

        // If the required topic was not found then break the loop and indicate
        // that the tracker is not suitable
        if (!topic_found)
        {
          tracker_suitable = false;
          break;
        }
      }

      // Remove the tracker if its last filter does not contain the required topic type
      if (!tracker_suitable)
      {
        trackers.erase(tracker_it);
      }
      else
      {
        tracker_it++;
      }
    }

    // If no tracker was suitable, then throw an error
    if (trackers.empty())
    {
      throw CREATE_ERROR(error::Code::NO_TRACKERS_FOUND, "No trackers found for the requested topic types");
    }
  }

  return trackers;
}

/*
 * Load tracker callback
 */
void ContextManager::loadTrackerCb(temoto_2::LoadTracker::Request& req,
                                   temoto_2::LoadTracker::Response& res)
{
  TEMOTO_INFO_STREAM("Received a request: \n" << req << std::endl);

  try
  {
    // Get the trackers that follow the requested criteria
    TrackerInfoPtrs trackers = findTrackers(req);

    TEMOTO_DEBUG_STREAM("Found the requested tracker category.");

    /*
     * Choose a tracker based on a TODO metric. Currently the tracker that has the
     * highest reliability value is chosen
     */
    TrackerInfoPtr tracker = *(std::max_element(trackers.begin(), trackers.end(),
                               [](const TrackerInfoPtr lhs, const TrackerInfoPtr rhs)
                               {
                                 return lhs->reliability_.getReliability() <
                                        rhs->reliability_.getReliability();
                               }));

    TEMOTO_DEBUG_STREAM("The following tracking method was chosen: \n" << tracker->toString().c_str());

    // Get the id of the pipe, if provided
    std::string pipe_id = req.pipe_id;

    // Create a new pipe id if it was not specified
    if (pipe_id == "")
    {
      // Create a unique pipe identifier string
      pipe_id = "pipe_" + std::to_string(pipe_id_generator_.generateID())
              + "_at_" + common::getTemotoNamespace();
    }

    /*
     * Build the pipe based on the number of filters. If the pipe
     * contains only one filter, then there are no constraints on
     * the ouptut topic types. But if the pipe contains multiple filters
     * then each preceding filter has to provide the topics that are
     * required by the proceding filter
     */
    TopicContainer required_topics;

    if (tracker->getPipeSize() > 1)
    {
      // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
      // for-loop, then it crashes on the second loop. Not sure why.
      std::set<std::string> req_tops = tracker->getPipe().at(1).required_input_topic_types_;

      // Loop over requested topics
      for (auto& topic : req_tops)
      {
        required_topics.addOutputTopicType(topic);
      }
    }
    else
    {
      // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
      // for-loop, then it crashes on the second loop. Not sure why.
      std::set<std::string> req_tops = tracker->getPipe().at(0).required_output_topic_types_;

      // Loop over requested topics
      for (auto& topic : req_tops)
      {
        required_topics.addOutputTopicType(topic);
      }
    }

    // Loop over the pipe
    std::vector<Filter> pipe = tracker->getPipe();

    // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
    std::vector<int> sub_resource_ids;

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

        // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
        sub_resource_ids.push_back(load_sensor_msg.response.rmp.resource_id);

        required_topics.setInputTopicsByKeyValue(load_sensor_msg.response.output_topics);

        // This line is necessary if the pipe size is 1
        required_topics.setOutputTopicsByKeyValue(load_sensor_msg.response.output_topics);
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
          // ... get the requirements for the output topic types from the proceding filter
          for (auto& topic_type : pipe.at(i+1).required_input_topic_types_)
          {
            required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/filter_" + std::to_string(i) + "/" + topic_type);
          }
        }
        else
        {
          // ... get the requirements for the output topics from own output topic requirements
          // TODO: throw if the "required_output_topic_types_" is empty
          for (auto& topic_type : pipe.at(i).required_output_topic_types_)
          {
            required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/filter_" + std::to_string(i) + "/" + topic_type);
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

        // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
        sub_resource_ids.push_back(load_algorithm_msg.response.rmp.resource_id);

        required_topics.setInputTopicsByKeyValue(load_algorithm_msg.response.output_topics);
      }
    }

    // Send the output topics of the last filter back via response
    res.output_topics = required_topics.outputTopicsAsKeyValues();

    // Add the tracker to allocated trackers + increase its reliability
    tracker->reliability_.adjustReliability();
    //allocated_trackers_[res.rmp.resource_id] = tracker;
    allocated_trackers_hack_[res.rmp.resource_id] = std::pair<TrackerInfoPtr, std::vector<int>>(tracker, sub_resource_ids);

    return;
  }
  catch (error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  throw CREATE_ERROR(error::Code::NO_TRACKERS_FOUND, "Could not find trackers for the requested category");
}

/*
 * Unload tracker callback
 */
void ContextManager::unloadTrackerCb(temoto_2::LoadTracker::Request& req, temoto_2::LoadTracker::Response& res)
{
  // Remove the tracker from the list of allocated trackers
  auto it = allocated_trackers_hack_.find(res.rmp.resource_id);
  if (it != allocated_trackers_hack_.end())
  {
    TEMOTO_DEBUG_STREAM("Erasing a tracker from the list of allocated trackers");
    allocated_trackers_hack_.erase(it);
  }
  else
  {
    throw CREATE_ERROR(error::Code::RESOURCE_NOT_FOUND, "Could not unload the tracker, because"
                       " it does not exist in the list of allocated trackers");
  }
}

/*
 * Load gesture callback
 */
void ContextManager::loadGestureCb(temoto_2::LoadGesture::Request& req,
                                   temoto_2::LoadGesture::Response& res)
{
  TEMOTO_INFO("Gesture requested.");
  TEMOTO_DEBUG("Using hardcoded specifiers[0]");

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.gesture_specifiers[0].type;

  // Call the sensor manager to arrange us a gesture sensor
  try
  {
    resource_manager_1_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                   sensor_manager::srv_name::SERVER,
                                                   msg);

    TEMOTO_DEBUG("Got a response: '%s'.", msg.response.rmp.message.c_str());
    res.package_name = msg.response.package_name;
    res.executable = msg.response.executable;
    res.topic = msg.response.output_topics.at(0).value;
  }
  catch (error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
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
        categorized_trackers_[tracker_category].push_back(std::make_shared<context_manager::TrackerInfo>(tracker_info));

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
  TEMOTO_DEBUG("Gesture unloaded.");
}

/*
 * TODO: use the generic tracker service
 */
void ContextManager::loadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                  temoto_2::LoadSpeech::Response& res)
{
  TEMOTO_INFO("Speech requested.");
  TEMOTO_DEBUG("Using hardcoded specifiers[0]");

  temoto_2::LoadSensor msg;
  msg.request.sensor_type = req.speech_specifiers[0].type;

  // Request a speech sensor from the Sensor Manager
  try
  {
    resource_manager_1_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                   sensor_manager::srv_name::SERVER,
                                                   msg);

    TEMOTO_DEBUG("Got a response: '%s'", msg.response.rmp.message.c_str());
    res.package_name = msg.response.package_name;
    res.executable = msg.response.executable;
    res.topic = msg.response.output_topics.at(0).value; // TODO deprecated
  }
  catch (error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

void ContextManager::unloadSpeechCb(temoto_2::LoadSpeech::Request& req,
                                    temoto_2::LoadSpeech::Response& res)
{
  TEMOTO_INFO("Speech unloaded.");
}

/*
 * Status callback 1
 */
void ContextManager::statusCb1(temoto_2::ResourceStatus& srv)
{
  /* TODO */
}

/*
 * Status callback 2
 */
void ContextManager::statusCb2(temoto_2::ResourceStatus& srv)
{
  TEMOTO_DEBUG("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    TEMOTO_DEBUG("A resource, that a running tracker depends on, has failed");

//    auto it = allocated_trackers_.find(srv.request.resource_id);
//    if (it != allocated_trackers_.end())

    int val = srv.request.resource_id;

    auto it = std::find_if(allocated_trackers_hack_.begin(), allocated_trackers_hack_.end(),
              [val](const std::pair<int, std::pair<TrackerInfoPtr, std::vector<int>>>& pair_in)
              {
                for (const auto& client_id : pair_in.second.second)
                {
                  if (client_id == val)
                  {
                    return true;
                  }
                }
                return false;
              });

    if (it != allocated_trackers_hack_.end())
    {
      TEMOTO_DEBUG("Tracker of type '%s' (pipe size: %d) has stopped working",
                   (it->second.first)->getType().c_str(),
                   (it->second.first)->getPipeSize());

      // Reduce the reliability of the tracker
      (it->second.first)->reliability_.adjustReliability(0);
    }
  }
}

}  // namespace context_manager
