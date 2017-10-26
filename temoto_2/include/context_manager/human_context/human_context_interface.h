#ifndef HUMAN_CONTEXT_INTERFACE_H
#define HUMAN_CONTEXT_INTERFACE_H

#include "core/common.h"

#include "base_task/task_errors.h"
#include "base_task/task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"
#include "common/interface_errors.h"

#include "std_msgs/Float32.h"
#include "human_msgs/Hands.h"

#include "context_manager/human_context/human_context_services.h"
#include "rmp/resource_manager.h"
#include <vector>

//#include <boost/function.hpp>

template <class Owner>
class HumanContextInterface
{
public:
  HumanContextInterface()
  {
  }

  void initialize(Task* task)
  {
    log_class_= "";
    log_subsys_ = "human_context_interface";
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/human_context_interface";
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<HumanContextInterface>>(new rmp::ResourceManager<HumanContextInterface>(name_, this));
  }

  void getGestures(std::vector<temoto_2::GestureSpecifier> gesture_specifiers,
                   void (Owner::*callback)(human_msgs::Hands), Owner* owner)
  {
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
    ROS_INFO("[HumanContextInterface::getGestures] subscribing to topic'%s'",
             srv_msg.response.topic.c_str());
    gesture_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, callback, owner);
  }

  void getSpeech(std::vector<temoto_2::SpeechSpecifier> speech_specifiers,
                 void (Owner::*callback)(std_msgs::String), Owner* owner)
  {
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
      ROS_INFO("getting speech");
      resource_manager_->template call<temoto_2::LoadSpeech>(
          human_context::srv_name::MANAGER, human_context::srv_name::SPEECH_SERVER, srv_msg);
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
    ROS_INFO("[HumanContextInterface::getSpeech] subscribing to topic'%s'",
             srv_msg.response.topic.c_str());
    speech_subscriber_ = nh_.subscribe(srv_msg.response.topic, 1000, callback, owner);
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
    }
    catch (...)
    {
      throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::CORE,
                                  error::Urgency::HIGH, prefix + " Failed to unload resources",
                                  ros::Time::now());
    }
  }

  ~HumanContextInterface()
  {
    // Let the context manager know, that task is finished and topics are unsubscribed
    stopAllocatedServices();
    gesture_subscriber_.shutdown();
    speech_subscriber_.shutdown();
  }

  const std::string& getName() const
  {
    return log_subsys_;
  }

private:
  std::unique_ptr<rmp::ResourceManager<HumanContextInterface>> resource_manager_;

  std::string name_; 
  std::string log_class_, log_subsys_, log_group_;

  ros::NodeHandle nh_;
  ros::Subscriber gesture_subscriber_;
  ros::Subscriber speech_subscriber_;

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
