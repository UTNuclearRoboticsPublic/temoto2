#ifndef HUMAN_CONTEXT_INTERFACE_H
#define HUMAN_CONTEXT_INTERFACE_H


#include "TTP/base_task/base_task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "human_msgs/Hands.h"
#include "leap_motion_controller/Set.h"

#include "context_manager/context_manager_services.h"
#include "rmp/resource_manager.h"
#include <vector>

namespace context_manager
{

template <class OwnerTask>
class ContextManagerInterface : public BaseSubsystem
{
public:

  typedef void (OwnerTask::*GestureCallbackType)(leap_motion_controller::Set);
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
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<ContextManagerInterface>>(new rmp::ResourceManager<ContextManagerInterface>(name_, this));

    // ensure that resource_manager was created
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    // register status callback function
    resource_manager_->registerStatusCb(&ContextManagerInterface::statusInfoCb);

    // Add object service client
    add_object_client_ = nh_.serviceClient<temoto_2::AddObjects>(context_manager::srv_name::SERVER_ADD_OBJECTS);
  }

  void getSpeech(std::vector<temoto_2::SpeechSpecifier> speech_specifiers, SpeechCallbackType callback, OwnerTask* obj)
  {
    task_speech_cb_ = callback;
    task_speech_obj_ = obj;

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the speech specifier and if successful, get
    // the name of the topic

    temoto_2::LoadSpeech srv_msg;
    srv_msg.request.speech_specifiers = speech_specifiers;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::LoadSpeech>(context_manager::srv_name::MANAGER
                                                            , context_manager::srv_name::SPEECH_SERVER
                                                            , srv_msg);
      allocated_speeches_.push_back(srv_msg);
    }
    catch (...)
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call service");
    }

    // Check if the request was satisfied
    // TODO: in future, catch code==0 exeption from RMP and rethrow from here
    if (srv_msg.response.rmp.code != 0)
    {
      throw FORWARD_ERROR(srv_msg.response.rmp.error_stack);
    }

    // Subscribe to the topic that was provided by the "Context Manager"
    TEMOTO_DEBUG("%s subscribing to topic'%s'", prefix.c_str(), srv_msg.response.topic.c_str());
    speech_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, task_speech_cb_, task_speech_obj_);
  }

  void getGesture(std::vector<temoto_2::GestureSpecifier> gesture_specifiers, GestureCallbackType callback, OwnerTask* obj)
  {
    task_gesture_cb_ = callback;
    task_gesture_obj_ = obj;

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_2::LoadGesture srv_msg;
    srv_msg.request.gesture_specifiers = gesture_specifiers;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::LoadGesture>(
          context_manager::srv_name::MANAGER, context_manager::srv_name::GESTURE_SERVER, srv_msg);
      allocated_gestures_.push_back(srv_msg);
    }
    catch (...)
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call the server");
    }

    // Check if the request was satisfied
    // TODO: in future, catch code==0 exeption from RMP and rethrow from here
    if (srv_msg.response.rmp.code != 0)
    {
      throw FORWARD_ERROR(srv_msg.response.rmp.error_stack);
    }

    // Subscribe to the topic that was provided by the "Context Manager"
    // TODO: This should be a vector of subsctibers, not just one.
    TEMOTO_INFO("%s subscribing to topic'%s'", prefix.c_str(), srv_msg.response.topic.c_str());
    gesture_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, task_gesture_cb_, task_gesture_obj_);
  }

  void addWorldObjects(const std::vector<temoto_2::ObjectContainer>& objects)
  {
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

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
        throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Direction method unspecified");
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
    if (add_obj_srvmsg.response.rmp.code != 0)
    {
      throw FORWARD_ERROR(add_obj_srvmsg.response.rmp.error_stack);
    }
  }

  void addWorldObjects(const temoto_2::ObjectContainer& object)
  {
    std::vector<temoto_2::ObjectContainer> objects;
    objects.push_back(object);
    addWorldObjects(objects);
  }



  bool stopAllocatedServices()
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    try
    {
      // remove all connections, which were created via call() function
      resource_manager_->unloadClients();
      allocated_gestures_.clear();
      allocated_speeches_.clear();
    }
    catch (...)
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to unload resources");
    }
  }

  void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    TEMOTO_DEBUG("%s status info was received", prefix.c_str());
    TEMOTO_DEBUG_STREAM(srv.request);
    // if any resource should fail, just unload it and try again
    // there is a chance that sensor manager gives us better sensor this time
    if (srv.request.status_code == rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("Human context interface detected a sensor failure. Unloading and "
                                "trying again");
      auto gest_it = std::find_if(allocated_gestures_.begin(), allocated_gestures_.end(),
                                  [&](const temoto_2::LoadGesture& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      auto speech_it = std::find_if(allocated_speeches_.begin(), allocated_speeches_.end(),
                                  [&](const temoto_2::LoadSpeech& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });
      if (speech_it != allocated_speeches_.end())
      {
        TEMOTO_DEBUG("Unloading speech");
        resource_manager_->unloadClientResource(speech_it->response.rmp.resource_id);
        TEMOTO_DEBUG("Asking the same speech again");
        if (!resource_manager_->template call<temoto_2::LoadSpeech>(
                context_manager::srv_name::MANAGER, context_manager::srv_name::SPEECH_SERVER, *speech_it))
        {
          throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call service");
        }

        if (speech_it->response.rmp.code == 0)
        {
          // Replace subscriber
          TEMOTO_DEBUG("Replacing subscriber new topic'%s'",
                   speech_it->response.topic.c_str());
          gesture_subscriber_.shutdown();
          gesture_subscriber_ = nh_.subscribe(speech_it->response.topic, 1000, task_speech_cb_, task_speech_obj_);
        }
        else
        {
          throw FORWARD_ERROR(speech_it->response.rmp.error_stack);
        }
      }


      if (gest_it != allocated_gestures_.end())
      {
        TEMOTO_DEBUG("Unloading gesture");
        resource_manager_->unloadClientResource(gest_it->response.rmp.resource_id);
        TEMOTO_DEBUG("Asking the same gesture again");
        if (!resource_manager_->template call<temoto_2::LoadGesture>(
                context_manager::srv_name::MANAGER, context_manager::srv_name::GESTURE_SERVER, *gest_it))
        {
          throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call service");
        }

        if (gest_it->response.rmp.code == 0)
        {
          // Replace subscriber
          TEMOTO_DEBUG("Replacing subscriber new topic'%s'",
                   gest_it->response.topic.c_str());
          gesture_subscriber_.shutdown();
          gesture_subscriber_ = nh_.subscribe(gest_it->response.topic, 1000, task_gesture_cb_, task_gesture_obj_);
        }
        else
        {
          throw FORWARD_ERROR(gest_it->response.rmp.error_stack);
        }
      }

      if (gest_it != allocated_gestures_.end() && speech_it != allocated_speeches_.end())
      {
        TEMOTO_ERROR("%s Got resource_id that is not registered in this interface.", prefix.c_str());
      }
    }
  }


  ~ContextManagerInterface(){}

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  GestureCallbackType task_gesture_cb_;
  SpeechCallbackType task_speech_cb_;

  //TODO: Currently task is able to give callback functions which are only bind to itself.
  // arbitrary objects should be allowed in future.
  OwnerTask* task_speech_obj_;
  OwnerTask* task_gesture_obj_;
  
  std::unique_ptr<rmp::ResourceManager<ContextManagerInterface>> resource_manager_;

  std::string name_; 
  TTP::BaseTask* task_;

  ros::NodeHandle nh_;
  ros::Subscriber gesture_subscriber_;
  ros::Subscriber speech_subscriber_;
  ros::ServiceClient add_object_client_;

  std::vector<temoto_2::LoadGesture> allocated_gestures_;
  std::vector<temoto_2::LoadSpeech> allocated_speeches_;

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace
#endif
