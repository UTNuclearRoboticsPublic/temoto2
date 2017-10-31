#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <map>
#include <moveit/move_group_interface/move_group_interface.h>
#include "common/temoto_id.h"

namespace robot_manager
{
class Robot
{
public:
  Robot(const std::string& robot_name);
  void addPlanningGroup(const std::string& planning_group_name);
  void removePlanningGroup(const std::string& planning_group_name);
  void plan(const std::string& planning_group_name);
  inline const std::string& getName() const
  {
    return robot_name_;
  }

  inline void setId(const temoto_id::ID robot_id)
  {
    robot_id_ = robot_id;
  }

  inline const temoto_id::ID getId() const
  {
    return robot_id_;
  }

private:
  std::string log_class_, log_subsys_, log_group_;
  std::string robot_name_;
  temoto_id::ID robot_id_;  ///< Resource_id of the launchfile process that was started for this
                           ///robot
  std::map<std::string, moveit::planning_interface::MoveGroupInterface> planning_groups_;
};
}

#endif
