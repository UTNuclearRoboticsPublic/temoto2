#include "robot_manager/robot.h"
#include "core/common.h"

namespace robot_manager
{
Robot::Robot(RobotInfoPtr robot_info_ptr)
  : robot_info_ptr_(robot_info_ptr), is_plan_valid_(false)
{
  log_class_ = "Robot";
  log_subsys_ = "robot_manager";
  log_group_ = "robot_manager";
  // TODO: this is temporary solution for default ur[X]_moveit_config groups
  if (isLocal())
  {
    addPlanningGroup("manipulator");
  }
}

Robot::~Robot()
{
    TEMOTO_DEBUG("Robot destructed");
}

void Robot::addPlanningGroup(const std::string& planning_group_name)
{
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group(
      new moveit::planning_interface::MoveGroupInterface(planning_group_name));
  //  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlannerId("ESTkConfigDefault");
  group->setNumPlanningAttempts(2);
  group->setPlanningTime(5);

  // Playing around with tolerances
  group->setGoalPositionTolerance(0.001);
  group->setGoalOrientationTolerance(0.001);
  group->setGoalJointTolerance(0.001);

  planning_groups_.emplace(planning_group_name, std::move(group));
}

void Robot::removePlanningGroup(const std::string& planning_group_name)
{
  planning_groups_.erase(planning_group_name);
}

void Robot::plan(const std::string& planning_group_name, geometry_msgs::PoseStamped& target_pose)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  auto group_it =
      planning_groups_.find(planning_group_name);  ///< Will throw if group does not exist
  if (group_it != planning_groups_.end())
  {
    group_it->second->setStartStateToCurrentState();
    group_it->second->setPoseTarget(target_pose);
    int* bbb = new int(35); 
    char* c = (char*)bbb;
    is_plan_valid_ = static_cast<bool>( group_it->second->plan(last_plan) );
    TEMOTO_DEBUG("%s Plan %s", prefix.c_str(), is_plan_valid_ ? "FOUND" : "FAILED");
  }
  else
  {
    TEMOTO_ERROR("%s Planning group '%s' was not found.", prefix.c_str(),
                 planning_group_name.c_str());
  }
}

void Robot::execute(const std::string& planning_group_name)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  moveit::planning_interface::MoveGroupInterface::Plan empty_plan;
  if (!is_plan_valid_)
  {
    TEMOTO_ERROR("%s Unable to execute group '%s' without a plan.", prefix.c_str(),
                 planning_group_name.c_str());
    return;
  }
  auto group_it =
      planning_groups_.find(planning_group_name);  ///< Will throw if group does not exist
  if (group_it != planning_groups_.end())
  {
    bool success;
    group_it->second->setStartStateToCurrentState();
    group_it->second->setRandomTarget();
    success = static_cast<bool>(group_it->second->execute(last_plan));
    TEMOTO_DEBUG("%s Execution %s", prefix.c_str(), success ? "SUCCESSFUL" : "FAILED");
  }
  else
  {
    TEMOTO_ERROR("%s Planning group '%s' was not found.", prefix.c_str(),
                 planning_group_name.c_str());
  }
}

std::string Robot::getName() const
{
  if(robot_info_ptr_)
  {
    return robot_info_ptr_->getName();
  }
}

bool Robot::isLocal() const
{
  if (robot_info_ptr_) 
  {
    return robot_info_ptr_->getTemotoNamespace() == ::common::getTemotoNamespace();
  }
 return true; // some default that should never reached. 
}

}
