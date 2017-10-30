#include "robot_manager/robot_manager.h"
#include "process_manager/process_manager_services.h"
//#include "rviz/QMap.h"
//#include "rviz/yaml_config_reader.h"
//#include "rviz/config.h"

namespace robot_manager
{
RobotManager::RobotManager() : resource_manager_(srv_name::MANAGER, this)
{
  active_robot_id_ = temoto_id::UNASSIGNED_ID;
  log_class_ = "";
  log_subsys_ = "robot_manager";
  log_group_ = "robot_manager";
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  
  // Fire up servers
  server_load_ = nh_.advertiseService(robot_manager::srv_name::SERVER_LOAD, &RobotManager::loadCb, this);
  server_plan_ = nh_.advertiseService(robot_manager::srv_name::SERVER_PLAN, &RobotManager::planCb, this);
  server_exec_ = nh_.advertiseService(robot_manager::srv_name::SERVER_EXECUTE, &RobotManager::execCb, this);
  server_get_rviz_cfg_ = nh_.advertiseService(robot_manager::srv_name::SERVER_GET_RVIZ_CONFIG, &RobotManager::getRvizConfigCb, this);
//  server_set_target_ = nh_.advertiseServer<temoto_2::RobotSetTarget>(robot_manager::srv_name::SERVER_SET_TARGET, &RobotManager::setTargetCb, this);

  
  TEMOTO_INFO("Robot manager is ready.");
}

bool RobotManager::loadCb(temoto_2::RobotLoad::Request& req,
                                 temoto_2::RobotLoad::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_INFO("%s Starting to load robot '%s'...", prefix.c_str(), req.robot_name.c_str());

  // check if any robot is loaded and if we have to unload it
  if (active_robot_id_ != temoto_id::UNASSIGNED_ID)
  {
    if (active_load_msg_.request == req)
    {
      TEMOTO_DEBUG("%s The robot '%s' is already loaded.", prefix.c_str(), req.robot_name.c_str());
      res = active_load_msg_.response;
      return true;
    }

    // a different robot is being requested, unload the one which is currently active
    resource_manager_.unloadClientResource(active_robot_id_);
    temoto_2::RobotLoad empty_load_msg;
    active_load_msg_ = empty_load_msg;
  }

  // Create an empty message that will be filled out by "findRobot" function
  temoto_2::LoadProcess load_proc_msg;

  // Find the suitable robot and fill the process manager service request
  auto pkg_ptr = findRobot(load_proc_msg.request, req.robot_name);
  if (pkg_ptr)
  {
    TEMOTO_INFO("%s Loading a robot: '%s', '%s', '%s', reliability %.3f", prefix.c_str(),
                load_proc_msg.request.action.c_str(), load_proc_msg.request.package_name.c_str(),
                load_proc_msg.request.executable.c_str(), pkg_ptr->getReliability());
    if (resource_manager_.call<temoto_2::LoadProcess>(
            process_manager::srv_name::MANAGER, process_manager::srv_name::SERVER, load_proc_msg))
    {
      TEMOTO_DEBUG("%s Call to ProcessManager was sucessful.", prefix.c_str());
      robots_.emplace(req.robot_name, moveit::planning_interface::MoveGroupInterface("manipulator"));
      //robot.setPlannerId("RRTConnectkConfigDefault");
      //robot.setNumPlanningAttempts(2);
      //robot.setPlanningTime(3);

      //// Playing around with tolerances
      //robot.setGoalPositionTolerance(0.001);
      //robot.setGoalOrientationTolerance(0.001);
      //robot.setGoalJointTolerance(0.001);
    }
    else
    {
      TEMOTO_ERROR("%s Failed to call to ProcessManager.", prefix.c_str());
      return true;
    }

    res.code = load_proc_msg.response.rmp.code;
    res.message = load_proc_msg.response.rmp.message;
    active_load_msg_.request = req;
    active_load_msg_.response = res;

    // Increase or decrease reliability depending on the return code
    (res.code == 0) ? pkg_ptr->adjustReliability(1.0) : pkg_ptr->adjustReliability(0.0);

    TEMOTO_DEBUG("%s Robot '%s' load.", prefix.c_str(), req.robot_name.c_str());
  }
  else
  {
    res.code = 1;
    res.message = "Robot manager did not find a suitable robot.";
    TEMOTO_ERROR("%s %s", prefix.c_str(), res.message.c_str());
  }
  return true;
}

bool RobotManager::planCb(temoto_2::RobotPlan::Request& req,
                                   temoto_2::RobotPlan::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s PLANNING...", prefix.c_str());

  moveit::planning_interface::MoveGroupInterface move_group_interface_ =
      moveit::planning_interface::MoveGroupInterface("manipulator");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
 // move_group_interface_.setPoseTarget(target_pose1);
  move_group_interface_.setPlannerId("RRTConnectkConfigDefault");
  move_group_interface_.setRandomTarget();
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  ROS_INFO("[move_robot/main] Planning frame: %s", move_group_interface_.getPlanningFrame().c_str());
  ROS_INFO("[move_robot/main] End effector link: %s", move_group_interface_.getEndEffectorLink().c_str());
  ROS_INFO("[move_robot/main] End effector: %s", move_group_interface_.getEndEffector().c_str());
  ROS_INFO("[move_robot/main] Goal position tolerance is: %.6f", move_group_interface_.getGoalPositionTolerance());
  ROS_INFO("[move_robot/main] Goal orientation tolerance is: %.6f", move_group_interface_.getGoalOrientationTolerance());
  ROS_INFO("[move_robot/main] Goal joint tolerance is: %.6f", move_group_interface_.getGoalJointTolerance());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group_interface_.plan(my_plan);

  TEMOTO_DEBUG("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  TEMOTO_DEBUG("%s DONE PLANNING...", prefix.c_str());
  return true;
}

bool RobotManager::execCb(temoto_2::RobotExecute::Request& req,
                                temoto_2::RobotExecute::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_INFO("%s EXECUTING...", prefix.c_str());
  return true;
}

bool RobotManager::getRvizConfigCb(temoto_2::RobotGetRvizConfig::Request& req,
                                temoto_2::RobotGetRvizConfig::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_INFO("%s GETTING CONFIG...", prefix.c_str());

  std::string  rviz_conf = "{Planning Request: {Planning Group: manipulator}}";

  res.config = rviz_conf;
  res.code = 0;
  return true;
}

//bool RobotManager::setTargetCb(temoto_2::LoadSpeech::Request& req,
//                                  temoto_2::LoadSpeech::Response& res)
//{
//  std::string prefix = "[RobotManager::unloadSpeechCb]:";
//  TEMOTO_INFO("%s Speech unloaded.", prefix.c_str());
//}

PackageInfoPtr RobotManager::findRobot(temoto_2::LoadProcess::Request& req, const std::string& robot_name)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  // Local list of devices that follow the requirements
  std::vector<PackageInfoPtr> candidates;
  std::vector<PackageInfoPtr> candidatesLocal;

  // Find packages where type matches specified name
  for (auto pkg_ptr : pkg_infos_)
  {
    if (pkg_ptr->getType() == robot_name && pkg_ptr->getLaunchables().size() > 0)
    {
      candidates.push_back(pkg_ptr);
    }
  }

  // If the list is empty, leave the req empty
  if (candidates.empty())
  {
    return NULL;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](PackageInfoPtr& pkg_ptr1, PackageInfoPtr& pkg_ptr2) {
              return pkg_ptr1->getReliability() > pkg_ptr2->getReliability();
            });

    // Get the name of the package and first launchable
    req.package_name = candidates[0]->getName();
    req.action = "roslaunch";
    req.executable = candidates[0]->getLaunchables().begin()->first;
    return candidates[0];
  }

}  // namespace robot_manager
