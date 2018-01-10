#include "robot_manager/robot.h"
#include "ros/package.h"


namespace robot_manager
{
Robot::Robot(RobotConfigPtr config, rmp::ResourceManager<RobotManager>& resource_manager, BaseSubsystem& b)
  : config_(config), resource_manager_(resource_manager), is_plan_valid_(false), BaseSubsystem(b)
{
  class_name_ = "Robot";

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
  if (config_->getRobotFeatures().size())
  {
    TEMOTO_ERROR(
        "Robot is missing features. Please specify urdf, manipulation, navigation or gripper sections in the configuration file.");
    TEMOTO_ERROR_STREAM(config_->toString() << "  " << config_->getRobotFeatures().size());
    // \TODO:: throw
    return;
  }

  // Load robot's main launch file
  // It should bring up joint_state/robot publishers and hw specific nodes
  loadUrdf();
  loadHardware();

  if (hasFeature(FeatureType::MANIPULATION))
  {
    loadManipulation();
  }
  
  if (hasFeature(FeatureType::NAVIGATION))
  {
    loadNavigation();
  }

}

// Load robot's hardware
void Robot::loadHardware()
{
  RobotFeature& ftr = config_->getRobotFeature(FeatureType::HARDWARE);
  try
  {
    temoto_id::ID res_id = rosExecute(ftr.getPackageName(), ftr.getExecutable());
    TEMOTO_DEBUG("HARDWARE resource id: %d", res_id);
    ftr.setResourceId(res_id);

    // Wait for robot/joint states become available.
    // std::string cmd_vel_topic = config->getAbsRobotNamespace() + "/cmd_vel";
    //waitForTopic(cmd_vel_topic, res_id);
    ftr.setLoaded(true);
    TEMOTO_DEBUG("Robot HARDWARE loaded.");
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

void Robot::waitForParam(const std::string& param, temoto_id::ID interrupt_res_id)
{
//\TODO: add 30 sec timeout protection.

  while (!nh_.hasParam(param))
  {
    TEMOTO_DEBUG("Waiting for %s ...", param.c_str());
    if (resource_manager_.hasFailed(interrupt_res_id))
    {
      //throw CREATE_ERROR(robot_error::SERVICE_STATUS_FAIL, "Loading interrupted. A FAILED status was received from process manager.");
    }
    ros::Duration(0.2).sleep();
  }
}

void Robot::waitForTopic(const std::string& topic, temoto_id::ID interrupt_res_id)
{
  //\TODO: add 30 sec timeout protection.
  while (!isTopicAvailable(topic))
  {
    TEMOTO_DEBUG("Waiting for %s ...", topic.c_str());
    if (resource_manager_.hasFailed(interrupt_res_id))
    {
      throw CREATE_ERROR(error::Code::SERVICE_STATUS_FAIL, "Loading interrupted. A FAILED status was received from process manager.");
    }
    ros::Duration(0.2).sleep();
  }
  TEMOTO_DEBUG("%s Topic '%s' was found.", topic.c_str());
}

bool Robot::isTopicAvailable(const std::string& topic)
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  auto it = std::find_if(
      master_topics.begin(), master_topics.end(),
      [&](ros::master::TopicInfo& master_topic) -> bool { return master_topic.name == topic; });
  return it != master_topics.end();
}

// Load robot's urdf
void Robot::loadUrdf()
{
  try
  {
    FeatureURDF& ftr = config_->getFeatureURDF();
    std::string urdf_path = '/' + ros::package::getPath(ftr.getPackageName()) + '/' + ftr.getExecutable();
    temoto_id::ID res_id = rosExecute("temoto_2", "urdf_loader.py", urdf_path);
    TEMOTO_DEBUG("URDF resource id: %d", res_id);
    ftr.setResourceId(res_id);

    std::string robot_desc_param = config_->getAbsRobotNamespace() + "/robot_description";
    waitForParam(robot_desc_param, res_id);
    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'urdf' loaded.");
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load move group and move group interfaces
void Robot::loadManipulation()
{
  try
  {
    FeatureManipulation& ftr = config_->getFeatureManipulation();
    temoto_id::ID res_id = rosExecute(ftr.getPackageName(), ftr.getExecutable());
    TEMOTO_DEBUG("Manipulation resource id: %d", res_id);
    ftr.setResourceId(res_id);

    // Add planning groups
    // TODO: read groups from srdf automatically
    for (auto group : config_->getFeatureManipulation().getPlanningGroups())
    {
      TEMOTO_DEBUG("Adding planning group %s", group.c_str());
      addPlanningGroup(group);
    }

    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'manipulation' loaded.");
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load robot driver that will publish joint states and robot state
void Robot::loadManipulationDriver()
{
  try
  {
    FeatureManipulation& ftr = config_->getFeatureManipulation();
    temoto_id::ID res_id = rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable());
    TEMOTO_DEBUG("Manipulation driver resource id: %d", res_id);
    ftr.setDriverResourceId(res_id);

    std::string joint_states_topic = config_->getAbsRobotNamespace() + "/joint_states";
    waitForTopic(joint_states_topic, res_id);

    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'manipulation driver' loaded.");
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load Move Base
void Robot::loadNavigation()
{
  FeatureNavigation& ftr = config_->getFeatureNavigation();
  temoto_id::ID res_id = rosExecute(ftr.getPackageName(), ftr.getExecutable());
  TEMOTO_DEBUG("Navigation resource id: %d", res_id);
  ftr.setResourceId(res_id);

  ftr.setLoaded(true);
  TEMOTO_DEBUG("Feature 'navigation' loaded.");
}
catch (error::ErrorStack& error_stack)
{
  throw FORWARD_ERROR(error_stack);
  }
}


// Load robot driver that will publish odom
void Robot::loadNavigationDriver()
{
  try
  {
    FeatureNavigation& ftr = config_->getFeatureNavigation();
    temoto_id::ID res_id = rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable());
    TEMOTO_DEBUG("Manipulation driver resource id: %d", res_id);
    ftr.setDriverResourceId(res_id);

    std::string odom_topic = config_->getAbsRobotNamespace() + "/odom";
    waitForTopic(odom_topic, res_id);

    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'navigation driver' loaded.");
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load Gripper
void Robot::loadGripper()
{

}


temoto_id::ID Robot::rosExecute(const std::string& package_name, const std::string& executable,
                       const std::string& args)
{
  temoto_2::LoadProcess load_proc_srvc;
  load_proc_srvc.request.package_name = package_name;
  load_proc_srvc.request.ros_namespace = config_->getAbsRobotNamespace(); //Execute in robot namespace
  load_proc_srvc.request.action = process_manager::action::ROS_EXECUTE;
  load_proc_srvc.request.executable = executable;
  load_proc_srvc.request.args = args;

  try
  {
    resource_manager_.call<temoto_2::LoadProcess>(
        process_manager::srv_name::MANAGER, process_manager::srv_name::SERVER, load_proc_srvc);
  }
  catch(error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  return load_proc_srvc.response.rmp.resource_id;
}


void Robot::addPlanningGroup(const std::string& planning_group_name)
{
  //Prepare robot description path and a nodehandle, which is in robot's namespace
  std::string rob_desc = config_->getAbsRobotNamespace() + "/robot_description";
  ros::NodeHandle mg_nh(config_->getAbsRobotNamespace());
  moveit::planning_interface::MoveGroupInterface::Options opts(planning_group_name, rob_desc, mg_nh);
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group(
      new moveit::planning_interface::MoveGroupInterface(opts));
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
  auto group_it =
      planning_groups_.find(planning_group_name);  ///< Will throw if group does not exist
  if (group_it != planning_groups_.end())
  {
    group_it->second->setStartStateToCurrentState();
    group_it->second->setPoseTarget(target_pose);
    is_plan_valid_ = static_cast<bool>( group_it->second->plan(last_plan) );
    TEMOTO_DEBUG("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  }
  else
  {
    TEMOTO_ERROR("Planning group '%s' was not found.", planning_group_name.c_str());
  }
}

void Robot::execute(const std::string& planning_group_name)
{
  moveit::planning_interface::MoveGroupInterface::Plan empty_plan;
  if (!is_plan_valid_)
  {
    TEMOTO_ERROR("Unable to execute group '%s' without a plan.", planning_group_name.c_str());
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
    TEMOTO_DEBUG("Execution %s",  success ? "SUCCESSFUL" : "FAILED");
  }
  else
  {
    TEMOTO_ERROR("Planning group '%s' was not found.", planning_group_name.c_str());
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
  std::string act_rob_ns = config_->getAbsRobotNamespace();
  YAML::Node info;
  YAML::Node rviz = info["RViz"];

  // RViz options

  if (hasFeature(FeatureType::URDF))
  {
    rviz["urdf"]["robot_description"] = act_rob_ns + "/robot_description";
  }

  if (hasFeature(FeatureType::MANIPULATION))
  {
    rviz["manipulation"]["move_group_ns"] = act_rob_ns;
  }

  if (hasFeature(FeatureType::NAVIGATION))
  {
    rviz["navigation"]["move_base_ns"] = act_rob_ns;
  }
  
  return YAML::Dump(info);
}
}
