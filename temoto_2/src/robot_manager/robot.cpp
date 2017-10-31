#include "robot_manager/robot.h"
#include "core/common.h"

namespace robot_manager
{
Robot::Robot(const std::string& robot_name)
  : robot_name_(robot_name), robot_id_(temoto_id::UNASSIGNED_ID)
{
  log_class_ = "Robot";
  log_subsys_ = "robot_manager";
  log_group_ = "robot_manager";
  // TODO: this is temporary solution for default ur[X]_moveit_config groups
  // additional constructor with planning grou
  addPlanningGroup("manipulator");
}

void Robot::addPlanningGroup(const std::string& planning_group_name)
{
moveit::planning_interface::MoveGroupInterface group = moveit::planning_interface::MoveGroupInterface(planning_group_name);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setNumPlanningAttempts(2);
  group.setPlanningTime(3);
  
  // Playing around with tolerances
  group.setGoalPositionTolerance(0.001);
  group.setGoalOrientationTolerance(0.001);
  group.setGoalJointTolerance(0.001);

  group.setRandomTarget();

  planning_groups_.emplace(planning_group_name, group);
}

void Robot::removePlanningGroup(const std::string& planning_group_name)
{
  planning_groups_.erase(planning_group_name);
}

void Robot::plan(const std::string& planning_group_name)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  auto& group = planning_groups_.at(planning_group_name);  ///< Will throw if group does not exist
  group.setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = group.plan(my_plan);
  TEMOTO_DEBUG("%s Visualizing plan 1 (pose goal) %s", prefix.c_str(), success ? "" : "FAILED");
}
}
