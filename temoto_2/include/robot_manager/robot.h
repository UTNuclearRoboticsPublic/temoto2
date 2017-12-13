#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <map>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include "common/temoto_id.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include "robot_manager/robot_config.h"
#include "robot_manager/robot_manager.h"
#include "robot_manager/robot_feature.h"

namespace robot_manager
{

// Forward declaration
class RobotManager;

class Robot
{
public:
  Robot(RobotConfigPtr config_, rmp::ResourceManager<RobotManager>& resource_manager);
  virtual ~Robot();
  void addPlanningGroup(const std::string& planning_group_name);
  void removePlanningGroup(const std::string& planning_group_name);
  void plan(const std::string& planning_group_name, geometry_msgs::PoseStamped& target_pose);

  void execute(const std::string& planning_group_name);

  std::string getName() const
  {
    return config_->getName();
  }

  RobotConfigPtr getConfig()
  {
    return config_;
  }

  bool isLocal() const;

  inline bool hasFeature(FeatureType type) const
  {
    for (auto& feature : features_)
    {
      if (feature.getType() == type)
      {
        return true;
      }
    }
    return false;
  }

  // return all the information required to visualize this robot
  std::string getVizInfo();

private:
  void load();
  void loadHardware();
  void waitForHardware();
  void loadUrdf();
  void loadManipulation();
  void loadNavigation();
  void loadGripper();

  void rosExecute(const std::string& package_name, const std::string& executable,
                  const std::string& args, temoto_2::LoadProcess::Response& res);

  // Resource IDs for robot's features
  temoto_id::ID rid_urdf_;
  temoto_id::ID rid_manipulation_;
  temoto_id::ID rid_navigation_;
  temoto_id::ID rid_gripper_;

  // General
  std::string log_class_, log_subsys_, log_group_;

  // Robot configuration
  RobotConfigPtr config_;

  // Resource Manager
  rmp::ResourceManager<RobotManager>& resource_manager_;

  // Enabled features for this robot
  std::set<RobotFeature> features_;

  // Manipulation related
  bool is_plan_valid_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan;
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>>
      planning_groups_;
};
}

#endif
