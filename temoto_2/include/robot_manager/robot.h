#ifndef ROBOT_H
#define ROBOT_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include "robot_manager/robot_config.h"
#include "robot_manager/robot_manager.h"
#include "robot_manager/robot_features.h"
#include <string>
#include <map>
#include <vector>

namespace robot_manager
{

// Forward declaration
class RobotManager;

class Robot : public BaseSubsystem
{
public:
  Robot(RobotConfigPtr config_, rmp::ResourceManager<RobotManager>& resource_manager, BaseSubsystem& b);
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

  // return all the information required to visualize this robot
  std::string getVizInfo();

  bool hasResource(temoto_id::ID resource_id);

private:
  void load();
  void loadHardware();
  void waitForHardware();
  void loadUrdf();
  void loadManipulation();
  void loadManipulationDriver();
  void loadNavigation();
  void loadNavigationDriver();

  temoto_id::ID rosExecute(const std::string& package_name, const std::string& executable,
                  const std::string& args = "");

  void waitForParam(const std::string& param, temoto_id::ID interrupt_res_id);
  void waitForTopic(const std::string& topic, temoto_id::ID interrupt_res_id);

  bool isTopicAvailable(const std::string& topic);

  // General
  //  std::string log_class_, log_subsys_, log_group_;
  ros::NodeHandle nh_;

  // Robot configuration
  RobotConfigPtr config_;

  // Resource Manager
  rmp::ResourceManager<RobotManager>& resource_manager_;

  // Manipulation related
  bool is_plan_valid_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan;
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>>
      planning_groups_;
};
}

#endif
