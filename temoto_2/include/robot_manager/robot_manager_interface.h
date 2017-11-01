#ifndef ROBOT_MANAGER_INTERFACE_H
#define ROBOT_MANAGER_INTERFACE_H

#include "core/common.h"

#include "TTP/base_task/task_errors.h"
#include "TTP/base_task/base_task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"
#include "common/interface_errors.h"

#include "robot_manager/robot_manager_services.h"
#include "rmp/resource_manager.h"

#include <vector>
#include <string>

template <class OwnerTask>
class RobotManagerInterface
{
public:
  RobotManagerInterface(TTP::BaseTask* task) : task_(task)
  {
  }

  void initialize()
  {
    log_class_ = "";
    log_subsys_ = "robot_manager_interface";
    log_group_ = "interfaces." + task_->getPackageName();
    name_ = task_->getName() + "/robot_manager_interface";

    // create resource manager
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<RobotManagerInterface>>(
        new rmp::ResourceManager<RobotManagerInterface>(name_, this));

    // ensure that resource_manager was created
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // register status callback function
    // resource_manager_->registerStatusCb(&RobotManagerInterface::statusInfoCb);
    client_load_ =
        nh_.serviceClient<temoto_2::RobotLoad>(robot_manager::srv_name::SERVER_LOAD);
    client_plan_ =
        nh_.serviceClient<temoto_2::RobotPlan>(robot_manager::srv_name::SERVER_PLAN);
    client_exec_ =
        nh_.serviceClient<temoto_2::RobotExecute>(robot_manager::srv_name::SERVER_EXECUTE);
    client_rviz_cfg_ =
        nh_.serviceClient<temoto_2::RobotGetRvizConfig>(robot_manager::srv_name::SERVER_GET_RVIZ_CONFIG);
    client_set_target_ =
        nh_.serviceClient<temoto_2::RobotSetTarget>(robot_manager::srv_name::SERVER_SET_TARGET);
  }

  void loadRobot(std::string robot_name = "")
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_2::RobotLoad load_srv;
    load_srv.request.robot_name = robot_name;
    if(client_load_.call(load_srv))
    {
      TEMOTO_DEBUG("%s Call successful", prefix.c_str());
    }
    else
    {
      TEMOTO_ERROR("%s Service call failed", prefix.c_str());
    }
  }

  void plan()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotPlan msg;
    msg.request.use_default_target = true;

    if (!client_plan_.call(msg) || msg.response.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::ROBOT_MANAGER, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + msg.response.message, ros::Time::now());
    }
  }

  void plan(const geometry_msgs::PoseStamped& pose)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotPlan msg;
    msg.request.use_default_target = false;
    client_plan_.call(msg);
  }

  void execute()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotExecute msg;
    client_exec_.call(msg);
  }

  std::string getMoveitRvizConfig()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotGetRvizConfig msg;

    if (!client_rviz_cfg_.call(msg) || msg.response.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::ROBOT_MANAGER, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + msg.response.message, ros::Time::now());
    }
    return msg.response.config;
  }

  void setTarget(std::string target_type)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotSetTarget msg;
    msg.request.target_type = target_type;
    if (!client_set_target_.call(msg) || msg.response.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::ROBOT_MANAGER, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + msg.response.message, ros::Time::now());
    }
  }

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

  ~RobotManagerInterface()
  {
    // Shutdown robot manager clients.
    client_load_.shutdown();
    client_plan_.shutdown();
    client_exec_.shutdown();
    client_rviz_cfg_.shutdown();
    client_set_target_.shutdown();
  }

private:
  std::string name_;
  std::string log_class_, log_subsys_, log_group_;
  TTP::BaseTask* task_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_rviz_cfg_;
  ros::ServiceClient client_set_target_;

  std::unique_ptr<rmp::ResourceManager<RobotManagerInterface>> resource_manager_;
};

#endif
