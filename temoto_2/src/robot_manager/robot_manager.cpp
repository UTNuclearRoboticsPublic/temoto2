#include "ros/package.h"
#include "robot_manager/robot_manager.h"
#include "process_manager/process_manager_services.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace robot_manager
{
RobotManager::RobotManager()
  : BaseSubsystem("robot_manager", error::Subsystem::ROBOT_MANAGER, __func__)
  , resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &RobotManager::syncCb, this)
  , mode_(modes::AUTO)
{

  //\TODO:Remove, deprecated
  log_class_ = this->class_name_;
  log_subsys_ = this->subsystem_name_;

  // Start the server for loading/unloading robots as resources
  resource_manager_.addServer<temoto_2::RobotLoad>(srv_name::SERVER_LOAD, &RobotManager::loadCb,
                                                   &RobotManager::unloadCb);
  resource_manager_.registerStatusCb(&RobotManager::statusInfoCb);

  // Ask remote robot managers to send their robot config
  config_syncer_.requestRemoteConfigs();

  // Fire up additional servers for performing various actions on a robot.
  server_plan_ =
      nh_.advertiseService(robot_manager::srv_name::SERVER_PLAN, &RobotManager::planCb, this);
  server_exec_ =
      nh_.advertiseService(robot_manager::srv_name::SERVER_EXECUTE, &RobotManager::execCb, this);
  server_get_viz_cfg_ = nh_.advertiseService(robot_manager::srv_name::SERVER_GET_VIZ_INFO,
                                             &RobotManager::getVizInfoCb, this);
  server_set_target_ = nh_.advertiseService(robot_manager::srv_name::SERVER_SET_TARGET,
                                            &RobotManager::setTargetCb, this);
  server_set_mode_ = nh_.advertiseService(robot_manager::srv_name::SERVER_SET_MODE,
                                          &RobotManager::setModeCb, this);

  // Read the robot config for this manager.
  std::string yaml_filename =
      ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" + common::getTemotoNamespace() + ".yaml";
  // \TODO: check
  std::ifstream in(yaml_filename);
  YAML::Node yaml_config = YAML::Load(in);

  // Parse the Robots section
  if (yaml_config["Robots"])
  {
    local_configs_ = parseRobotConfigs(yaml_config);

    // Debug what was added
    for (auto& config : local_configs_)
    {
      TEMOTO_DEBUG("Added robot: '%s'.", config->getName().c_str());
      TEMOTO_DEBUG_STREAM("CONFIG: \n" << config->toString());
    }

    // Advertise the parsed local robots
    advertiseConfigs(local_configs_);
  }

  TEMOTO_INFO("Robot manager is ready.");
}

void RobotManager::loadLocalRobot(RobotConfigPtr config, temoto_id::ID resource_id)
{
  if (!config)
  {
    throw CREATE_ERROR(error::Code::NULL_PTR, "config == NULL");
  }

  try
  {
    active_robot_ = std::make_shared<Robot>(config, resource_manager_, *this);
    loaded_robots_.emplace(resource_id, active_robot_);
    config->adjustReliability(1.0);
    advertiseConfig(config);
    TEMOTO_DEBUG("Robot '%s' loaded.", config->getName().c_str());
  }
  catch (error::ErrorStack& error_stack)
  {
    //\TODO: Should we adjust reliability for only certain type of errors?
    config->adjustReliability(0.0);
    advertiseConfig(config);
    throw FORWARD_ERROR(error_stack);
  }
}

void RobotManager::loadCb(temoto_2::RobotLoad::Request& req, temoto_2::RobotLoad::Response& res)
{
  TEMOTO_INFO("Starting to load robot '%s'...", req.robot_name.c_str());

  // Find the suitable robot and fill the process manager service request
  auto config = findRobot(req.robot_name, local_configs_);
  if (config)
  {
    try
    {
      loadLocalRobot(config, res.rmp.resource_id);
      res.rmp.code = rmp::status_codes::OK;
      res.rmp.message = "Robot sucessfully loaded.";
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    catch (...)
    {
      TEMOTO_ERROR_STREAM("Failed to load local robot: Unknown exception.");
      return;
    }
    return;
  }

  // Try to find suitable candidate from remote managers
  config = findRobot(req.robot_name, remote_configs_);
  if (config)
  {
    try
    {
      temoto_2::RobotLoad load_robot_srvc;
      load_robot_srvc.request.robot_name = req.robot_name;
      TEMOTO_INFO("RobotManager is forwarding request: '%s'", req.robot_name.c_str());

      resource_manager_.call<temoto_2::RobotLoad>(robot_manager::srv_name::MANAGER,
                                                  robot_manager::srv_name::SERVER_LOAD,
                                                  load_robot_srvc,
                                                  rmp::FailureBehavior::NONE,
                                                  config->getTemotoNamespace());
      TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
      res.rmp = load_robot_srvc.response.rmp;
      active_robot_ = std::make_shared<Robot>(config, resource_manager_, *this);
      loaded_robots_.emplace(load_robot_srvc.response.rmp.resource_id, active_robot_);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    catch (...)
    {
      throw CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Exception occured while creating Robot "
                                                           "object.");
    }
    return;
  }
  else
  {
    // no local nor remote robot found
    throw CREATE_ERROR(error::Code::ROBOT_NOT_FOUND,"Robot manager did not find a suitable robot.");
  }
}

void RobotManager::unloadCb(temoto_2::RobotLoad::Request& req, temoto_2::RobotLoad::Response& res)
{
  TEMOTO_DEBUG("ROBOT '%s' unloading...", req.robot_name.c_str());
  TEMOTO_WARN_STREAM(req);
  TEMOTO_WARN_STREAM(res);

//  ros::Duration(5).sleep();
for (const auto& r : loaded_robots_)
{
  TEMOTO_WARN_STREAM(r.first);
}

  // search for the robot based on its resource id, remove from map,
  // and clear active_robot_ if the unloaded robot was active.
  auto it = loaded_robots_.find(res.rmp.resource_id);
  if (it != loaded_robots_.end())
  {
    TEMOTO_WARN("REMOVING ROBOT");
    if (active_robot_ == it->second)
    {
      active_robot_ = NULL;
    }
    loaded_robots_.erase(it);
  }
  TEMOTO_DEBUG("ROBOT '%s' unloaded.", req.robot_name.c_str());
}

void RobotManager::syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload)
{
  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseConfigs(local_configs_);
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node yaml_config = YAML::Load(payload.data);
    RobotConfigs configs = parseRobotConfigs(yaml_config);

    // TODO hold remote stuff in a map or something keyed by namespace
    for (auto& config : configs)
    {
      config->setTemotoNamespace(msg.temoto_namespace);
    }

    for (auto& config : configs)
    {
      // Check if robot config has to be added or updated
      auto it = std::find_if(remote_configs_.begin(), remote_configs_.end(),
                             [&](const RobotConfigPtr& ri) -> bool { return *ri == *config; });
      if (it != remote_configs_.end())
      {
        TEMOTO_DEBUG("Updating remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
        *it = config;  // overwrite found entry
      }
      else
      {
        TEMOTO_DEBUG("Adding remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
        remote_configs_.push_back(config);
      }
    }
  }
}

void RobotManager::advertiseConfig(RobotConfigPtr config)
{
  // publish all local robots
  YAML::Node yaml_config;
  yaml_config["Robots"].push_back(config->getYAMLConfig());
  PayloadType payload;
  payload.data = YAML::Dump(yaml_config);
  config_syncer_.advertise(payload);
}

void RobotManager::advertiseConfigs(RobotConfigs configs)
{
  // publish all local robots
  YAML::Node yaml_config;
  for (auto& config : configs)
  {
    yaml_config["Robots"].push_back(config->getYAMLConfig());
  }

  // send to other managers if there is anything to send
  if (yaml_config.size())
  {
    PayloadType payload;
    payload.data = YAML::Dump(yaml_config);
    config_syncer_.advertise(payload);
  }
}

RobotConfigs RobotManager::parseRobotConfigs(const YAML::Node& yaml_config)
{
  RobotConfigs configs;

  if (!yaml_config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("Unable to parse 'Robots' key from config.");
    return configs;
  }

  YAML::Node robots_node = yaml_config["Robots"];
  if (!robots_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of robots.");
    // TODO Throw
    return configs;
  }

  TEMOTO_DEBUG("Parsing %lu robots.", robots_node.size());

  // go over each robot node in the sequence
  for (YAML::const_iterator node_it = robots_node.begin(); node_it != robots_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("Unable to parse the robot config. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      RobotConfig config(*node_it, *this);
      if (std::count_if(configs.begin(), configs.end(),
                        [&](const RobotConfigPtr& ri) { return *ri == config; }) == 0)
      {
        // OK, this is unique config, add it to the configs.
        configs.emplace_back(std::make_shared<RobotConfig>(config));
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of robot '%s'.", config.getName().c_str());
      }
    }
    catch (...)
    {
      TEMOTO_WARN("Failed to parse RobotConfig from config.");
      continue;
    }
  }
  return configs;
}

bool RobotManager::planCb(temoto_2::RobotPlan::Request& req, temoto_2::RobotPlan::Response& res)
{
  TEMOTO_DEBUG("PLANNING...");
  if (active_robot_)
  {
    if (active_robot_->isLocal())
    {
      if (req.use_default_target)
      {
        geometry_msgs::PoseStamped pose;
        default_pose_mutex_.lock();
        pose = default_target_pose_;
        default_pose_mutex_.unlock();
        active_robot_->plan(req.planning_group, pose);
      }
      else
      {
        active_robot_->plan("manipulator", req.target_pose);
      }
      TEMOTO_DEBUG("DONE PLANNING...");
      res.message = "Planning sent to MoveIt";
      res.code = rmp::status_codes::OK;
    }
    else
    {
      // This robot is present in a remote robotmanager, forward the planning command to there.
      std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                          robot_manager::srv_name::SERVER_PLAN;
      ros::ServiceClient client_plan = nh_.serviceClient<temoto_2::RobotPlan>(topic);
      temoto_2::RobotPlan fwd_plan_srvc;
      fwd_plan_srvc.request = req;
      fwd_plan_srvc.response = res;
      if (client_plan.call(fwd_plan_srvc))
      {
        TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
        res = fwd_plan_srvc.response;
      }
      else
      {
        TEMOTO_ERROR("Call to remote RobotManager service failed.");
        res.message = "Call to remote RobotManager service failed.";
        res.code = rmp::status_codes::FAILED;
      }
    }
  }
  else
  {
    TEMOTO_ERROR("Unable to plan, because the robot is not loaded.");
    res.message = "Unable to plan, because the robot is not loaded.";
    res.code = rmp::status_codes::FAILED;
  }

  return true;
}

bool RobotManager::execCb(temoto_2::RobotExecute::Request& req,
                          temoto_2::RobotExecute::Response& res)
{
  TEMOTO_INFO("EXECUTING...");
  if (active_robot_)
  {
    if (active_robot_->isLocal())
    {
      active_robot_->execute("manipulator");
      TEMOTO_DEBUG("DONE EXECUTING...");
      res.message = "Execute command sent to MoveIt";
      res.code = rmp::status_codes::OK;
    }
    else
    {
      // This robot is present in a remote robotmanager, forward the command to there.
      std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                          robot_manager::srv_name::SERVER_EXECUTE;
      ros::ServiceClient client_exec = nh_.serviceClient<temoto_2::RobotExecute>(topic);
      temoto_2::RobotExecute fwd_exec_srvc;
      fwd_exec_srvc.request = req;
      fwd_exec_srvc.response = res;
      if (client_exec.call(fwd_exec_srvc))
      {
        TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
        res = fwd_exec_srvc.response;
      }
      else
      {
        TEMOTO_ERROR("Call to remote RobotManager service failed.");
        res.message = "Call to remote RobotManager service failed.";
        res.code = rmp::status_codes::FAILED;
      }
    }
  }
  else
  {
    TEMOTO_ERROR("Unable to execute, because the robot is not loaded.");
  }
  return true;
}

bool RobotManager::getVizInfoCb(temoto_2::RobotGetVizInfo::Request& req,
                                temoto_2::RobotGetVizInfo::Response& res)
{
  TEMOTO_INFO("GETTING visualization info...");
  // Search for the loaded robot, when its name is specified.
  if (req.robot_name != "")
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_id::ID, RobotPtr> p) -> bool {
                                   return p.second->getName() == req.robot_name;
                                 });
    if (robot_it != loaded_robots_.end())
    {
      res.info = robot_it->second->getVizInfo();
    }
    else
    {
      res.error_stack = CREATE_ERROR(error::Code::ROBOT_NOT_LOADED,
                                     "The requested robot '%s' is not loaded.", req.robot_name);
      res.code == rmp::status_codes::FAILED;
      return true;
    }
  }
  else
  {
    // Robot name is not specified, try to use the active robot.
    if (active_robot_)
    {
      res.info = active_robot_->getVizInfo();
    }
    else
    {
      res.error_stack =
          CREATE_ERROR(error::Code::ROBOT_NOT_LOADED, "No loaded robots found.", req.robot_name);
      res.code == rmp::status_codes::FAILED;
      return true;
    }
  }
  res.code = rmp::status_codes::OK;
  return true;
}

bool RobotManager::setTargetCb(temoto_2::RobotSetTarget::Request& req,
                               temoto_2::RobotSetTarget::Response& res)
{
  TEMOTO_INFO("Looking for target of type '%s'", req.target_type.c_str());

  // TODO: Only "hand" type is experimentally implemented
  if (req.target_type == "hand")
  {
    std::vector<temoto_2::GestureSpecifier> gesture_specifiers;
    temoto_2::GestureSpecifier gesture_specifier;
    gesture_specifier.dev = "device";
    gesture_specifier.type = "hand";
    //    gesture_specifiers.push_back(gesture_specifier);

    try
    {
      hand_srv_msg_.request.gesture_specifiers.push_back(gesture_specifier);
      resource_manager_.call<temoto_2::LoadGesture>(
          context_manager::srv_name::MANAGER, context_manager::srv_name::GESTURE_SERVER,
          hand_srv_msg_, rmp::FailureBehavior::NONE);
      TEMOTO_DEBUG("Subscribing to '%s'", hand_srv_msg_.response.topic.c_str());
      target_pose_sub_ =
        nh_.subscribe(hand_srv_msg_.response.topic, 1, &RobotManager::targetPoseCb, this);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }
  return true;
}

bool RobotManager::setModeCb(temoto_2::RobotSetMode::Request& req,
                             temoto_2::RobotSetMode::Response& res)
{
  TEMOTO_INFO("SET MODE...");
  // input validation
  if (req.mode != modes::AUTO && req.mode != modes::NAVIGATION && req.mode != modes::MANIPULATION)
  {
    TEMOTO_ERROR("Mode '%s' is not supported.", req.mode.c_str());
    res.message = "Mode is not supported.";
    res.code = rmp::status_codes::FAILED;
    return true;
  }

  if (active_robot_)
  {
    if (active_robot_->isLocal())
    {
      mode_ = req.mode;
      TEMOTO_DEBUG("Robot mode set to: %s...", mode_.c_str());
      res.message = "Robot mode set to '" + mode_ + "'.";
      res.code = rmp::status_codes::OK;
    }
    else
    {
      // This robot is present in a remote robotmanager, forward the command to there.
      std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                          robot_manager::srv_name::SERVER_SET_MODE;
      ros::ServiceClient client_mode = nh_.serviceClient<temoto_2::RobotSetMode>(topic);
      temoto_2::RobotSetMode fwd_mode_srvc;
      fwd_mode_srvc.request = req;
      fwd_mode_srvc.response = res;
      if (client_mode.call(fwd_mode_srvc))
      {
        TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
        res = fwd_mode_srvc.response;
      }
      else
      {
        TEMOTO_ERROR("Call to remote RobotManager service failed.");
        res.message = "Call to remote RobotManager service failed.";
        res.code = rmp::status_codes::FAILED;
      }
    }
  }
  else
  {
    TEMOTO_ERROR("Unable to set mode, because the robot is not loaded.");
  }
  return true;
}

// Take palm pose of whichever hand is present, prefer left_hand.
// Store the pose in a class member for later use when planning is requested.
void RobotManager::targetPoseCb(const leap_motion_controller::Set& set)
{
  if (set.left_hand.is_present)
  {
    default_pose_mutex_.lock();
    default_target_pose_ = set.left_hand.palm_pose;
    default_pose_mutex_.unlock();
  }
  else if (set.right_hand.is_present)
  {
    default_pose_mutex_.lock();
    default_target_pose_ = set.right_hand.palm_pose;
    default_pose_mutex_.unlock();
  }
}

void RobotManager::statusInfoCb(temoto_2::ResourceStatus& srv)
{
  TEMOTO_DEBUG("status info was received");
  TEMOTO_DEBUG_STREAM(srv.request);
  // if any resource should fail, just unload it and try again
  // there is a chance that sensor manager gives us better sensor this time
  if (srv.request.status_code == rmp::status_codes::FAILED &&
      srv.request.resource_id == hand_srv_msg_.response.rmp.resource_id)
  {
    TEMOTO_WARN("Robot manager detected a hand sensor failure. Unloading and "
                "trying again");
    try
    {
      resource_manager_.unloadClientResource(hand_srv_msg_.response.rmp.resource_id);
    }
    catch(error::ErrorStack& error_stack)
    {
      TEMOTO_ERROR_STREAM(error_stack);
    }

    // retry with previous request
    try
    {
      resource_manager_.call<temoto_2::LoadGesture>(context_manager::srv_name::MANAGER,
                                                    context_manager::srv_name::GESTURE_SERVER,
                                                    hand_srv_msg_,
                                                    rmp::FailureBehavior::NONE);
      TEMOTO_DEBUG("Subscribing to '%s'", hand_srv_msg_.response.topic.c_str());
      target_pose_sub_ =
          nh_.subscribe(hand_srv_msg_.response.topic, 1, &RobotManager::targetPoseCb, this);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  // Check if any of the allocated robots has failed
  // Currently we simply remove the loaded robot if it failed
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    // was it a remote robot
    if (loaded_robots_.erase(srv.request.resource_id))
    {
      TEMOTO_DEBUG("Removed remote robot, because its status failed.");
      return;
    }

    // check if it was a resource related to a robot feature has failed.
    // unload the robot
    for (auto it = loaded_robots_.begin(); it != loaded_robots_.end(); ++it)
    {
      if (it->second->hasResource(srv.request.resource_id))
      {
        RobotConfigPtr config = it->second->getConfig();
        config->adjustReliability(0.0);
        YAML::Node yaml_config;
        yaml_config["Robots"].push_back(config->getYAMLConfig());
        PayloadType payload;
        payload.data = YAML::Dump(yaml_config);
        std::cout<<payload<<std::endl;
        config_syncer_.advertise(payload);
        loaded_robots_.erase(it);
        break;
      }
    }
  }
}

RobotConfigPtr RobotManager::findRobot(const std::string& robot_name, const RobotConfigs& configs)
{
  // Local list of devices that follow the requirements
  RobotConfigs candidates;

  // If robot name is unspecified, pick the best one from all configs.
  if (robot_name == "")
  {
    candidates = configs;
  }
  else
  {
    // Find the robot that matches the "name" criteria
    auto it = std::copy_if(configs.begin(), configs.end(), std::back_inserter(candidates),
        [&](const RobotConfigPtr& s) { return s->getName() == robot_name; });
  }

  // If the list is empty, leave the req empty
  if (candidates.empty())
  {
    return NULL;
  }

  std::sort(candidates.begin(), candidates.end(), [](RobotConfigPtr& rc1, RobotConfigPtr& rc2) {
    return rc1->getReliability() > rc2->getReliability();
  });

  // Get the name of the package and first launchable
  return candidates.front();
}

}  // namespace robot_manager
