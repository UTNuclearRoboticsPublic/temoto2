#ifndef HUMAN_CONTEXT_INTERFACE_H
#define HUMAN_CONTEXT_INTERFACE_H


#include "TTP/base_task/base_task.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/console_colors.h"
#include "temoto_core/common/topic_container.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "human_msgs/Hands.h"

#include "context_manager/context_manager_services.h"
#include "rmp/resource_manager.h"
#include <vector>

namespace context_manager
{

template <class OwnerTask>
class ContextManagerInterface : public temoto_core::BaseSubsystem
{
public:

  typedef void (OwnerTask::*SpeechCallbackType)(std_msgs::String);

  ContextManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(TTP::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/context_manager_interface";

    // create resource manager
    resource_manager_ = std::unique_ptr<temoto_core::rmp::ResourceManager<ContextManagerInterface>>(new temoto_core::rmp::ResourceManager<ContextManagerInterface>(name_, this));

    // ensure that resource_manager was created
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // register status callback function
    resource_manager_->registerStatusCb(&ContextManagerInterface::statusInfoCb);

    // Add object service client
    add_object_client_ = nh_.serviceClient<temoto_2::AddObjects>(context_manager::srv_name::SERVER_ADD_OBJECTS);
  }

  int getNumber(const int number)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Create a GetNumber service message
    temoto_2::GetNumber srv_msg;
    srv_msg.request.requested_int = number;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::GetNumber>( context_manager::srv_name::MANAGER
                                                           , context_manager::srv_name::GET_NUMBER_SERVER
                                                           , srv_msg);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return srv_msg.response.responded_int;
  }

  void getSpeech(std::vector<temoto_2::SpeechSpecifier> speech_specifiers, SpeechCallbackType callback, OwnerTask* obj)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    task_speech_cb_ = callback;
    task_speech_obj_ = obj;

    // Contact the "Context Manager", pass the speech specifier and if successful, get
    // the name of the topic

    temoto_2::LoadSpeech srv_msg;
    srv_msg.request.speech_specifiers = speech_specifiers;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::LoadSpeech>( context_manager::srv_name::MANAGER
                                                            , context_manager::srv_name::SPEECH_SERVER
                                                            , srv_msg);
      allocated_speeches_.push_back(srv_msg);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Subscribe to the topic that was provided by the "Context Manager"
    TEMOTO_DEBUG("subscribing to topic'%s'", srv_msg.response.topic.c_str());
    speech_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, task_speech_cb_, task_speech_obj_);
  }

  /**
   * @brief startTracker
   * @param tracker_category
   * @return
   */
  temoto_core::TopicContainer startTracker(std::string tracker_category)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Start filling out the LoadTracker message
    temoto_2::LoadTracker load_tracker_msg;
    load_tracker_msg.request.tracker_category = tracker_category;

    try
    {
      resource_manager_->template call<temoto_2::LoadTracker>(context_manager::srv_name::MANAGER_2,
                                                              context_manager::srv_name::TRACKER_SERVER,
                                                              load_tracker_msg);
      allocated_trackers_.push_back(load_tracker_msg);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    temoto_core::TopicContainer topics_to_return;
    topics_to_return.setOutputTopicsByKeyValue(load_tracker_msg.response.output_topics);

    return topics_to_return;
  }

  std::string trackObject(std::string object_name)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Start filling out the TrackObject message
    temoto_2::TrackObject track_object_msg;
    track_object_msg.request.object_name = object_name;

    try
    {
      resource_manager_->template call<temoto_2::TrackObject>(context_manager::srv_name::MANAGER,
                                                              context_manager::srv_name::TRACK_OBJECT_SERVER,
                                                              track_object_msg);

      allocated_track_objects_.push_back(track_object_msg);
      return track_object_msg.response.object_topic;
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief addWorldObjects
   * @param objects
   */
  void addWorldObjects(const std::vector<temoto_2::ObjectContainer>& objects)
  {
    // Check if this message contains the basic parameters
    // Does it have a name
    for (auto& object : objects)
    {
      if (object.name == "")
      {
        throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL , "The object is missing a name");
      }

      // Are the detection methods specified
      if (object.detection_methods.empty())
      {
        throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Detection method unspecified");
      }
    }

    temoto_2::AddObjects add_obj_srvmsg;
    add_obj_srvmsg.request.objects = objects;

    // Call the server
    if (!add_object_client_.call(add_obj_srvmsg))
    {
       throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call the server");
    }

    // Check the response code
    // TODO: First of all, transfer the RMP members straight to the request part.
    //       Then, instead of checkin the code, check the error stack.
    if (add_obj_srvmsg.response.rmp.code != 0)
    {
      throw FORWARD_ERROR(add_obj_srvmsg.response.rmp.error_stack);
    }
  }

  /**
   * @brief addWorldObjects
   * @param object
   */
  void addWorldObjects(const temoto_2::ObjectContainer& object)
  {
    std::vector<temoto_2::ObjectContainer> objects;
    objects.push_back(object);
    addWorldObjects(objects);
  }

  /**
   * @brief stopAllocatedServices
   * @return
   */
  bool stopAllocatedServices()
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    try
    {
      // remove all connections, which were created via call() function
      resource_manager_->unloadClients();
      allocated_speeches_.clear();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("Received status information");
    TEMOTO_DEBUG_STREAM(srv.request);
    // if any resource should fail, just unload it and try again
    // there is a chance that sensor manager gives us better sensor this time
    if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("Received a notification about a resource failure. Unloading and trying again");

      auto speech_it = std::find_if(allocated_speeches_.begin(), allocated_speeches_.end(),
                                  [&](const temoto_2::LoadSpeech& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      auto tracker_it = std::find_if(allocated_trackers_.begin(), allocated_trackers_.end(),
                                  [&](const temoto_2::LoadTracker& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      auto track_object_it = std::find_if(allocated_track_objects_.begin(), allocated_track_objects_.end(),
                                  [&](const temoto_2::TrackObject& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      if (speech_it != allocated_speeches_.end())
      {
        try
        {
          TEMOTO_DEBUG("Unloading speech");
          resource_manager_->unloadClientResource(speech_it->response.rmp.resource_id);
          TEMOTO_DEBUG("Asking the same speech again");
          resource_manager_->template call<temoto_2::LoadSpeech>(
              context_manager::srv_name::MANAGER, context_manager::srv_name::SPEECH_SERVER,
              *speech_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }

        // Replace subscriber
        TEMOTO_DEBUG("Replacing subscriber new topic'%s'",
                 speech_it->response.topic.c_str());
        speech_subscriber_.shutdown();
        speech_subscriber_ = nh_.subscribe(speech_it->response.topic, 1000, task_speech_cb_, task_speech_obj_);
      }

      // If the tracker was found then ...
      else if (tracker_it != allocated_trackers_.end())
      {
        try
        {
          // ... unload it and ...
          TEMOTO_DEBUG("Unloading the tracker");
          resource_manager_->unloadClientResource(tracker_it->response.rmp.resource_id);

          // ... copy the output topics from the response into the output topics
          // of the request (since the user still wants to receive the data on the same topics) ...
          tracker_it->request.output_topics = tracker_it->response.output_topics;
          tracker_it->request.pipe_id = tracker_it->response.pipe_id;

          // ... and load an alternative tracker. This call automatically
          // updates the response in allocated trackers vector

          TEMOTO_DEBUG_STREAM("Trying to load an alternative tracker");

          resource_manager_->template call<temoto_2::LoadTracker>(context_manager::srv_name::MANAGER_2,
                                                                  context_manager::srv_name::TRACKER_SERVER,
                                                                  *tracker_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }

      // If the tracker was found then ...
      else if (track_object_it != allocated_track_objects_.end())
      {
        try
        {
          // ... unload it and ...
          TEMOTO_DEBUG("Unloading the track object");
          resource_manager_->unloadClientResource(track_object_it->response.rmp.resource_id);

          TEMOTO_DEBUG_STREAM("Trying to resume tracking the " << track_object_it->request.object_name);

          resource_manager_->template call<temoto_2::TrackObject>(context_manager::srv_name::MANAGER,
                                                                  context_manager::srv_name::TRACK_OBJECT_SERVER,
                                                                  *track_object_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }

      if (speech_it != allocated_speeches_.end() &&
          tracker_it != allocated_trackers_.end() &&
          track_object_it != allocated_track_objects_.end())
      {
        throw CREATE_ERROR(error::Code::RESOURCE_NOT_FOUND, "Got resource_id that is not registered in this interface.");
      }
    }
  }


  ~ContextManagerInterface(){}

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  SpeechCallbackType task_speech_cb_;

  //TODO: Currently task is able to give callback functions which are only bind to itself.
  // arbitrary objects should be allowed in future.
  OwnerTask* task_speech_obj_;
  
  std::unique_ptr<temoto_core::rmp::ResourceManager<ContextManagerInterface>> resource_manager_;

  std::string name_; 
  TTP::BaseTask* task_;

  ros::NodeHandle nh_;
  ros::Subscriber speech_subscriber_;
  ros::ServiceClient add_object_client_;

  std::vector<temoto_2::LoadSpeech> allocated_speeches_;
  std::vector<temoto_2::LoadTracker> allocated_trackers_;
  std::vector<temoto_2::TrackObject> allocated_track_objects_;

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface()
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace
#endif
