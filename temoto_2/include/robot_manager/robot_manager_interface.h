#ifndef ROBOT_MANAGER_INTERFACE_H
#define ROBOT_MANAGER_INTERFACE_H

#include "TTP/base_task/base_task.h"
#include "common/temoto_id.h"
#include "common/console_colors.h"

#include "robot_manager/robot_manager_services.h"
#include "rmp/resource_manager.h"

#include <vector>
#include <string>

namespace robot_manager
{

template <class OwnerTask>
class RobotManagerInterface : public BaseSubsystem
{
public:
  RobotManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(TTP::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    
    name_ = task->getName() + "/robot_manager_interface";
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

    // create resource manager
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<RobotManagerInterface>>(
        new rmp::ResourceManager<RobotManagerInterface>(name_, this));

    // ensure that resource_manager was created
    validateInterface(prefix);

    // register status callback function
    // resource_manager_->registerStatusCb(&RobotManagerInterface::statusInfoCb);
//    client_load_ =
//        nh_.serviceClient<temoto_2::RobotLoad>(robot_manager::srv_name::SERVER_LOAD);
    client_plan_ =
        nh_.serviceClient<temoto_2::RobotPlan>(robot_manager::srv_name::SERVER_PLAN);
    client_exec_ =
        nh_.serviceClient<temoto_2::RobotExecute>(robot_manager::srv_name::SERVER_EXECUTE);
    client_viz_info_ =
        nh_.serviceClient<temoto_2::RobotGetVizInfo>(robot_manager::srv_name::SERVER_GET_VIZ_INFO);
    client_set_target_ =
        nh_.serviceClient<temoto_2::RobotSetTarget>(robot_manager::srv_name::SERVER_SET_TARGET);
  }

  void loadRobot(std::string robot_name = "")
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_2::RobotLoad load_srvc;
    load_srvc.request.robot_name = robot_name;
    try
    {
      resource_manager_->template call<temoto_2::RobotLoad>(
          robot_manager::srv_name::MANAGER, robot_manager::srv_name::SERVER_LOAD, load_srvc);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  void plan(std::string planning_group = "")
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

    temoto_2::RobotPlan msg;
    msg.request.use_default_target = true;
    msg.request.planning_group = planning_group;

    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void plan(const geometry_msgs::PoseStamped& pose, std::string planning_group = "")
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotPlan msg;
    msg.request.use_default_target = false;
    msg.request.target_pose = pose;
    msg.request.planning_group = planning_group;
    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void execute()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotExecute msg;
    if (!client_exec_.call(msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  std::string getMoveitRvizConfig()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotGetVizInfo msg;
    if (!client_viz_info_.call(msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
    return msg.response.info;
  }

  void setTarget(std::string target_type)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_2::RobotSetTarget msg;
    msg.request.target_type = target_type;
    if (!client_set_target_.call(msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if (!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }

  const std::string& getName() const
  {
    return name_;
  }

  ~RobotManagerInterface()
  {
    // Shutdown robot manager clients.
    client_load_.shutdown();
    client_plan_.shutdown();
    client_exec_.shutdown();
    client_viz_info_.shutdown();
    client_set_target_.shutdown();

    TEMOTO_DEBUG("RobotManagerInterface destroyed.");
  }

private:
  std::string name_;
  std::string log_class_, log_subsys_, log_group_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_viz_info_;
  ros::ServiceClient client_set_target_;

  std::unique_ptr<rmp::ResourceManager<RobotManagerInterface>> resource_manager_;
};

} // namespace
#endif
