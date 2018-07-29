/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *    This is the base task that every task has to inherit and implement
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef BASE_TASK_H
#define BASE_TASK_H

#include "ros/ros.h"
#include "common/base_subsystem.h"
#include "common/temoto_id.h"
#include "common/tools.h"
#include "TTP/task_descriptor.h"
#include "temoto_2/StopTaskMsg.h"
#include <boost/any.hpp>
#include <string>
#include <exception>

/*
 * basic log management, everything put under temoto_2.tasks for easier level control
 */
#define TASK_CONSOLE_PREFIX ROSCONSOLE_ROOT_LOGGER_NAME "."+::common::getTemotoNamespace()+".tasks." + this->getPackageName()
#define TASK_DEBUG(...) ROS_LOG(::ros::console::levels::Debug, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_INFO(...) ROS_LOG(::ros::console::levels::Info, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_WARN(...) ROS_LOG(::ros::console::levels::Warn, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_ERROR(...) ROS_LOG(::ros::console::levels::Error, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_DEBUG_STREAM(...)                                                                     \
  ROS_LOG_STREAM(::ros::console::levels::Debug, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_INFO_STREAM(...)                                                                      \
  ROS_LOG_STREAM(::ros::console::levels::Info, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_WARN_STREAM(...)                                                                      \
  ROS_LOG_STREAM(::ros::console::levels::Warn, TASK_CONSOLE_PREFIX, __VA_ARGS__)
#define TASK_ERROR_STREAM(...)                                                                     \
  ROS_LOG_STREAM(::ros::console::levels::Error, TASK_CONSOLE_PREFIX, __VA_ARGS__)


namespace TTP
{

class BaseTask : public BaseSubsystem
{
friend class TaskManager;

public:

  /**
   * @brief Task default constructor.
   */
  BaseTask() : task_package_name_("task_unknown")
  {}

  /**
   * @brief startTaskWrapped
   * @param task_interface
   * @return
   */
  void startTaskWrapped(TaskInterface task_interface)
  {
    try
    {
      startTask(task_interface);
      task_is_finished_ = true;
    }
    catch(error::ErrorStack& error_stack)
    {
      SEND_ERROR(FORWARD_ERROR(error_stack));
    }
    catch(std::exception& e)
    {
      SEND_ERROR(CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Caught unhandled std exception: " + std::string(e.what())));
    }
    catch(...)
    {
      SEND_ERROR(CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Caught unhandled exception."));
    }
  }

  /**
   * @brief startTask
   * @param task_interface
   * @return
   */
  virtual void startTask(TaskInterface task_interface) = 0;

  /**
   * @brief stopTask
   * @return
   */
  bool stopTask()
  {
    stop_task_ = true;
    return 0;
  }

  /**
   * @brief taskFinished
   * @return
   */
  bool taskFinished()
  {
    return task_is_finished_;
  }

  /**
   * @brief getStatus
   * @return
   */
  virtual std::string getStatus()
  {
    return "healthy";
  }

  /**
   * @brief getID
   * @return
   */
  TemotoID::ID getID() const
  {
    return task_id_;
  }

  /**
   * @brief getName
   * @return Unique name that consists of package name and task ID
   */
  std::string getName() const
  {
    std::stringstream ss;
    ss << getPackageName() << "_" << getID();
    return ss.str();
  }

  const std::string& getPackageName() const
  {
    return task_package_name_;
  }


  /**
   * @brief getSolution
   * @return
   */
  virtual std::vector<Subject> getSolution() = 0;

  /**
   * @brief ~Task Implemented virtual constructor. If it would not be implemented,
   * a magical undefined .so reference error will appear if the task is destructed
   */
  virtual ~BaseTask(){};

  std::vector<TTP::Subject> input_subjects;

  std::vector<TTP::Subject> output_subjects;

protected:

  std::string description;
  bool stop_task_ = false;
  bool task_is_finished_ = false;

private:

  std::string task_package_name_;

  /**
   * @brief task_n_
   */
  ros::NodeHandle task_nodehandle_;

  /**
   * @brief task_id_
   */
  TemotoID::ID task_id_ = TemotoID::UNASSIGNED_ID;

  /**
   * @brief setID
   * @param task_id
   */
  void setID( TemotoID::ID task_id )
  {
    task_id_ = task_id;
  }

};
}// END of TTP namespace
#endif
