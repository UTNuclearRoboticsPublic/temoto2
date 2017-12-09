#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <map>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include "common/temoto_id.h"
#include "robot_manager/robot_info.h"

namespace robot_manager
{
namespace capability
{
const std::string URDF = "urdf";
const std::string MANIPULATION = "manipulation";
const std::string NAVIGATION = "navigation";
}

class Robot
{
public:
  Robot(RobotInfoPtr robot_info_ptr);
  virtual ~Robot();
  void addPlanningGroup(const std::string& planning_group_name);
  void removePlanningGroup(const std::string& planning_group_name);
  void plan(const std::string& planning_group_name, geometry_msgs::PoseStamped& target_pose);

  void execute(const std::string& planning_group_name);

  std::string getName() const;

  RobotInfoPtr getRobotInfo()
  {
    return robot_info_ptr_;
  }

  bool isLocal() const;

  // return all the information required to visualize this robot
  std::string getVizInfo();

private:
  bool is_plan_valid_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan;
  std::string log_class_, log_subsys_, log_group_;
  std::string robot_name_;
  RobotInfoPtr robot_info_ptr_;

  /// robot
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>>
      planning_groups_;
  std::set<std::string> capabilities_;
};
}

#endif
