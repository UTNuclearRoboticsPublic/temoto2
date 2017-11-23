/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *    This is the base task that every task has to inherit and implement
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef BASE_TASK_H
#define BASE_TASK_H

#include <string>
#include "TTP/task_descriptor.h"
#include "base_error/base_error.h"
#include <boost/any.hpp>
#include "common/temoto_id.h"
#include "common/tools.h"
#include "temoto_2/StopTaskMsg.h"
#include "ros/ros.h"

/*
 *basic log management, everything put under temoto_2.tasks for easier level control
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

class BaseTask
{
friend class TaskManager;

public:

    /**
     * @brief Task default constructor.
     */
    BaseTask() : task_package_name_("task_unknown")
    {
    }

    /**
     * @brief startTask
     * @param task_interface
     * @return
     */
    virtual bool startTask(TaskInterface task_interface) = 0;

    /**
     * @brief pauseTask
     * @return
     */
    bool pauseTask();

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

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    std::vector<TTP::Subject> input_subjects;

    std::vector<TTP::Subject> output_subjects;

protected:

    std::string description;
    bool stop_task_ = false;

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