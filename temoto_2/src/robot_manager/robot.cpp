#include "robot_manager/robot.h"
#include "core/common.h"

namespace robot_manager
{
Robot::Robot(RobotConfigPtr config)
  : config_(config), is_plan_valid_(false)
{
  log_class_ = "Robot";
  log_subsys_ = "robot_manager";
  log_group_ = "robot_manager";
  if (isLocal())
  {
    load();
  }
}

Robot::~Robot()
{
    TEMOTO_DEBUG("Robot destructed");
}

void Robot::load()
{
    if (features_.size())
    {
      TEMOTO_ERROR("Loading failed! Robot has to have at least one of the following features "
                   "(URDF, manipulation, "
                   "navigation or gripper).");
      // \TODO:: throw
      return;
    }

    loadHardware();
    waitForHardware();

    if (hasFeature(feature::URDF))
    {
      loadUrdf();
    }
    
    if (hasFeature(feature::MANIPULATION))
    {
      loadManipulation();
    }

    if (hasFeature(feature::NAVIGATION))
    {
      loadNavigation();
    }

    if (hasFeature(feature::GRIPPER))
    {
      loadGripper();
    }

}

// Load robot's hardware
void Robot::loadHardware()
{

    // Load robot's main launch file
    // It should bring up joint_state/robot publishers and hw specific nodes

//    rosExecute(config->getName(), config->getPackageName(), config->getExecutable(), "", load_proc_res);

 // rosExecute("temoto_2", "urdf_loader.py", "");
}

// Load robot's urdf
void Robot::loadUrdf()
{

 // rosExecute("temoto_2", "urdf_loader.py", "");
}

// Load MoveIt! move group and move group interfaces
void Robot::loadManipulation()
{

  // for each planning group, add 
  addPlanningGroup("manipulator");
}

// Load Move Base
void Robot::loadNavigation()
{

}

// Load Gripper
void Robot::loadGripper()
{

}

// Wait for hardware. Poll parameter server until robot_description becomes available.
// \TODO: add 30 sec timeout protection.
void Robot::waitForHardware()
{
//  while (!nh_.hasParam('/' + config->getRobotNamespace() + "/robot_description"))
//  {
//    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
//    TEMOTO_DEBUG("%s Waiting for move group to be ready ...", prefix.c_str());
//    if (resource_manager_.hasFailed(res_id))
//    {
//      throw error::ErrorStackUtil(
//          robot_error::SERVICE_STATUS_FAIL, error::Subsystem::ROBOT_MANAGER, error::Urgency::MEDIUM,
//          prefix + "Loading interrupted. A FAILED status was received from process manager.");
//    }
//    ros::Duration(0.2).sleep();
//  }
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


bool Robot::isLocal() const
{
  if (config_) 
  {
    return config_->getTemotoNamespace() == ::common::getTemotoNamespace();
  }
 return true; // some default that should never reached. 
}

std::string Robot::getVizInfo()
{
  std::string act_rob_ns = config_->getRobotNamespace();
  YAML::Node info;
  YAML::Node rviz = info["RViz"];

  // RViz options

  if (hasFeature(feature::URDF))
  {
    rviz["urdf"]["robot_description"] = "/" + act_rob_ns + "/robot_description";
  }

  if (hasFeature(feature::MANIPULATION))
  {
    rviz["manipulation"]["move_group_ns"] = '/' + act_rob_ns;
  }

  if (hasFeature(feature::NAVIGATION))
  {
    rviz["navigation"]["move_base_ns"] = '/' + act_rob_ns;
  }
  
  return Dump(info);
}
}
