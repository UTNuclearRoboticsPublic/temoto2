#ifndef HUMAN_CONTEXT_INTERFACE_H
#define HUMAN_CONTEXT_INTERFACE_H

#include "core/common.h"

#include "TTP/base_task/task_errors.h"
#include "TTP/base_task/base_task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"
#include "common/interface_errors.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "human_msgs/Hands.h"
#include "leap_motion_controller/Set.h"

#include "context_manager/human_context/human_context_services.h"
#include "rmp/resource_manager.h"
#include <vector>

template <class OwnerTask>
class HumanContextInterface
{
public:

  typedef void (OwnerTask::*GestureCallbackType)(leap_motion_controller::Set);
  typedef void (OwnerTask::*SpeechCallbackType)(std_msgs::String);


  HumanContextInterface(TTP::BaseTask* task) : task_(task)
  {
  }

  void initialize()
  {
    log_class_= "";
    log_subsys_ = "human_context_interface";
    log_group_ = "interfaces." + task_->getPackageName();
    name_ = task_->getName() + "/human_context_interface";

    // create resource manager
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<HumanContextInterface>>(new rmp::ResourceManager<HumanContextInterface>(name_, this));

    // ensure that resource_manager was created
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // register status callback function
    resource_manager_->registerStatusCb(&HumanContextInterface::statusInfoCb);
  }

  void getGesture(std::vector<temoto_2::GestureSpecifier> gesture_specifiers, GestureCallbackType callback, OwnerTask* obj)
  {
    task_gesture_cb_ = callback;
    task_gesture_obj_ = obj;

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_2::LoadGesture srv_msg;
    srv_msg.request.gesture_specifiers = gesture_specifiers;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::LoadGesture>(
          human_context::srv_name::MANAGER, human_context::srv_name::GESTURE_SERVER, srv_msg);
      allocated_gestures_.push_back(srv_msg);
    }
    catch (...)
    {
      throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK,
                                  error::Urgency::MEDIUM, prefix + " Failed to call service",
                                  ros::Time::now());
    }

    // Check if the request was satisfied
    // TODO: in future, catch code==0 exeption from RMP and rethrow from here
    if (srv_msg.response.rmp.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + srv_msg.response.rmp.message, ros::Time::now());
    }

    // Subscribe to the topic that was provided by the "Context Manager"
    TEMOTO_INFO("%s subscribing to topic'%s'", prefix.c_str(), srv_msg.response.topic.c_str());
    gesture_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, task_gesture_cb_, task_gesture_obj_);
  }

  void getSpeech(std::vector<temoto_2::SpeechSpecifier> speech_specifiers, SpeechCallbackType callback, OwnerTask* obj)
  {
    task_speech_cb_ = callback;
    task_speech_obj_ = obj;

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the speech specifier and if successful, get
    // the name of the topic

    temoto_2::LoadSpeech srv_msg;
    srv_msg.request.speech_specifiers = speech_specifiers;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_2::LoadSpeech>(
          human_context::srv_name::MANAGER, human_context::srv_name::SPEECH_SERVER, srv_msg);
      allocated_speeches_.push_back(srv_msg);
    }
    catch (...)
    {
      throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK,
                                  error::Urgency::MEDIUM, prefix + " Failed to call service",
                                  ros::Time::now());
    }

    // Check if the request was satisfied
    // TODO: in future, catch code==0 exeption from RMP and rethrow from here
    if (srv_msg.response.rmp.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + srv_msg.response.rmp.message, ros::Time::now());
    }

    // Subscribe to the topic that was provided by the "Context Manager"
    TEMOTO_DEBUG("%s subscribing to topic'%s'", prefix.c_str(), srv_msg.response.topic.c_str());
    speech_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, task_speech_cb_, task_speech_obj_);
  }

  bool stopAllocatedServices()
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
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
      throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::CORE,
                                  error::Urgency::HIGH, prefix + " Failed to unload resources",
                                  ros::Time::now());
    }
  }



void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
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
                human_context::srv_name::MANAGER, human_context::srv_name::SPEECH_SERVER, *speech_it))
        {
          throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK,
                                      error::Urgency::MEDIUM, prefix + " Failed to call service",
                                      ros::Time::now());
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
          throw error::ErrorStackUtil(
              taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
              prefix + " Unsuccessful call to context manager: " + speech_it->response.rmp.message,
              ros::Time::now());
        }
      }


      if (gest_it != allocated_gestures_.end())
      {
        TEMOTO_DEBUG("Unloading gesture");
        resource_manager_->unloadClientResource(gest_it->response.rmp.resource_id);
        TEMOTO_DEBUG("Asking the same gesture again");
        if (!resource_manager_->template call<temoto_2::LoadGesture>(
                human_context::srv_name::MANAGER, human_context::srv_name::GESTURE_SERVER, *gest_it))
        {
          throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK,
                                      error::Urgency::MEDIUM, prefix + " Failed to call service",
                                      ros::Time::now());
        }

        if (gest_it->response.rmp.code == 0)
        {
          // Replace subscriber
          TEMOTO_DEBUG("Replacing subscriber new topic'%s'",
                   speech_it->response.topic.c_str());
          gesture_subscriber_.shutdown();
          gesture_subscriber_ = nh_.subscribe(speech_it->response.topic, 1000, task_gesture_cb_, task_gesture_obj_);
        }
        else
        {
          throw error::ErrorStackUtil(
              taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
              prefix + " Unsuccessful call to context manager: " + gest_it->response.rmp.message,
              ros::Time::now());
        }
      }

      if (gest_it != allocated_gestures_.end() && speech_it != allocated_speeches_.end())
      {
        TEMOTO_ERROR("%s Got resource_id that is not registered in this interface.", prefix.c_str());
      }
    }
  }



  ~HumanContextInterface()
  {
    // Let the context manager know, that task is finished and topics are unsubscribed
    //stopAllocatedServices();
    //gesture_subscriber_.shutdown();
    //speech_subscriber_.shutdown();
  }

  const std::string& getName() const
  {
    return log_subsys_;
  }

private:

  GestureCallbackType task_gesture_cb_;
  SpeechCallbackType task_speech_cb_;

  //TODO: Currently task is able to give callback functions which are only bind to itself.
  // arbitrary objects should be allowed in future.
  OwnerTask* task_speech_obj_;
  OwnerTask* task_gesture_obj_;
  
  std::unique_ptr<rmp::ResourceManager<HumanContextInterface>> resource_manager_;

  std::string name_; 
  std::string log_class_, log_subsys_, log_group_;
  TTP::BaseTask* task_;

  ros::NodeHandle nh_;
  ros::Subscriber gesture_subscriber_;
  ros::Subscriber speech_subscriber_;

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
      TEMOTO_ERROR("%s Interface is not initalized.", log_prefix.c_str());
      throw error::ErrorStackUtil(
          interface_error::NOT_INITIALIZED, error::Subsystem::TASK, error::Urgency::MEDIUM,
          log_prefix + " Interface is not initialized.", ros::Time::now());
    }
  }
};

#endif
