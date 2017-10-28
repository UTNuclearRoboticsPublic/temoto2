#ifndef ROBOT_MANAGER_INTERFACE_H
#define ROBOT_MANAGER_INTERFACE_H

#include "core/common.h"

#include "base_task/task_errors.h"
#include "base_task/task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"
#include "common/interface_errors.h"

#include "robot_manager/robot_manager_services.h"

#include <vector>

template <class OwnerTask>
class RobotManagerInterface
{
public:
  RobotManagerInterface(Task* task) : task_(task)
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
        nh_.serviceClient<temoto_2::RobotManagerLoad>(robot_manager::srv_name::SERVER_LOAD);
    client_plan_ =
        nh_.serviceClient<temoto_2::RobotManagerPlan>(robot_manager::srv_name::SERVER_PLAN);
    client_exec_ =
        nh_.serviceClient<temoto_2::RobotManagerExecute>(robot_manager::srv_name::SERVER_EXECUTE);
  }

  void loadRobot(std::string robot_name = "")
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_2::RobotLoad load_srv;
    load_srv.robot_name = robot_name;
    client_load_.call(load_srv);
  }

  void plan()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotPlan msg;
    msg.use_default_target = true;

    if (!client_plan_.call(msg) || srv_msg.code != 0)
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::ROBOT_MANAGER, error::Urgency::MEDIUM,
          prefix + " Service request failed: " + srv_msg.response.rmp.message, ros::Time::now());
    }
  }

  void plan(const std_msgs::Pose& pose)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotPlan msg;
    msg.use_default_target = false;
    client_plan_.call(msg);
  }

  void execute()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotExecute msg;
    msg.use_default_target = true;
    client_exec_.call(msg);
  }

  void execute(const std_msgs::Pose& pose)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotExecute msg;
    msg.use_default_target = false;
    client_exec_.call(msg);
  }

  ~RobotManagerInterface()
  {
    // Shutdown robot manager clients.
    client_load_.shutdown();
    client_plan_.shutdown();
    client_exec_.shutdown();
  }

private:
  std::string name_;
  std::string log_class_, log_subsys_, log_group_;
  Task* task_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_plan_;
};

#endif
