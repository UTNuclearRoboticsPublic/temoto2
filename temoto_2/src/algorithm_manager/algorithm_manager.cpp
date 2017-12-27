#include "ros/package.h"
#include "algorithm_manager/algorithm_manager.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace algorithm_manager
{

/*
 * Constructor
 */
AlgorithmManager::AlgorithmManager()
  : resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &AlgorithmManager::syncCb, this)
{
  class_name_ = __func__;
  subsystem_name_ = "algorithm_manager";
  subsystem_code_ = error::Subsystem::ALGORITHM_MANAGER;
  log_group_ = "algorithm_manager";
  error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);

  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  // Start the server
  resource_manager_.addServer<temoto_2::LoadAlgorithm>(srv_name::SERVER
                                                     , &AlgorithmManager::loadAlgorithmCb
                                                     , &AlgorithmManager::unloadAlgorithmCb);

  // Register callback for status info
  resource_manager_.registerStatusCb(&AlgorithmManager::statusCb);

  // Synchronize algorithm configurations
  config_syncer_.requestRemoteConfigs();

  // Read the algorithms for this manager.

  ///////////////////  TODO:  This parsing has to be done per package   //////////////////
  std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                              common::getTemotoNamespace() + ".yaml";

  std::ifstream in(yaml_filename);
  YAML::Node config = YAML::Load(in);

  if (config["Algorithms"])
  {
    local_algorithms_ = parseAlgorithms(config);

    for (auto& s : local_algorithms_)
    {
      TEMOTO_DEBUG("%s Added algorithm: '%s'.", prefix.c_str(), s->getName().c_str());
    }

    // notify other managers about our algorithms
    advertiseLocalAlgorithms();
  }
  else
  {
    TEMOTO_WARN("%s Failed to read '%s'. Verify that the file exists and the sequence of algorithms "
                "is listed under 'Algorithms' node.",
                prefix.c_str(), yaml_filename.c_str());
  }
  ///////////////////////////////////////////////////////////////////////////////////////

  TEMOTO_INFO_STREAM(prefix << " Algorithm manager is ready.");
}

/*
 * Status callback
 */
void AlgorithmManager::statusCb(temoto_2::ResourceStatus& srv)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s Received a status message", prefix.c_str());

  // If the status message indicates, that the algorithm has failed,
  // then adjust it's reliability
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_algorithms_.find(srv.request.resource_id);
    if (it != allocated_algorithms_.end())
    {
      TEMOTO_WARN("Algorithm failure detected, adjusting reliability.");
      it->second->adjustReliability(0.0);
      YAML::Node config;
      config["Algorithms"].push_back(*it->second);

      // Synchronize
      PayloadType payload;
      payload.data = Dump(config);
      config_syncer_.advertise(payload);
    }
  }
}

/*
 * Synchronization callback
 */
void AlgorithmManager::syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  // Advertise local algorithms
  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseLocalAlgorithms();
    return;
  }

  // Update/add remote algorithms
  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(payload.data);
    std::vector<AlgorithmInfoPtr> algorithms = parseAlgorithms(config);

    // TODO: Hold remote stuff in a map or something keyed by namespace
    // TODO: Temoto namespace can (doesn't have to) be contained in config
    for (auto& s : algorithms)
    {
      s->setTemotoNamespace(msg.temoto_namespace);
    }

    for (auto& algorithm : algorithms)
    {
      // Check if algorithm has to be added or updated
      auto it = std::find_if(remote_algorithms_.begin()
                           , remote_algorithms_.end()
                           , [&](const AlgorithmInfoPtr& ra)
                             {
                               return *ra == *algorithm;
                             });

      // Update the algorithm
      if (it != remote_algorithms_.end())
      {
        TEMOTO_DEBUG("%s Updating remote algorithm '%s' at '%s'.", prefix.c_str(),
            algorithm->getName().c_str(), algorithm->getTemotoNamespace().c_str());

        // Update by owerwriting the old entry
        *it = algorithm;
      }

      // Add new algorithm
      else
      {
        TEMOTO_DEBUG("%s Adding remote algorithm '%s' at '%s'.", prefix.c_str(),
            algorithm->getName().c_str(), algorithm->getTemotoNamespace().c_str());

        remote_algorithms_.push_back(algorithm);
      }
    }
  }
}

/*
 * Advertise local algorithm
 */
void AlgorithmManager::advertiseAlgorithm(AlgorithmInfoPtr algorithm_ptr)
{
    YAML::Node config;
    config["Sensors"].push_back(*algorithm_ptr);
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
}

/*
 * Advertise local algorithms
 */
void AlgorithmManager::advertiseLocalAlgorithms()
{
  // Publish all local algorithms
  YAML::Node config;
  for(auto& s : local_algorithms_)
  {
      config["Algorithms"].push_back(*s);
  }

  // Send to other managers if there is anything to send
  if(config.size())
  {
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
  }
}

/*
 * Load algorithm callback
 */
void AlgorithmManager::loadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req
                                     , temoto_2::LoadAlgorithm::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s received a request to load '%s': '%s', '%s'"
             , prefix.c_str()
             , req.algorithm_type.c_str()
             , req.package_name.c_str()
             , req.executable.c_str());

  // Try to find suitable candidate from local algorithms
  auto algorithm_ptr = findAlgorithm(req.algorithm_type, req.package_name, req.executable, local_algorithms_);
  if (algorithm_ptr)
  {
    // Local algorithm found, make a call to the local resource manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = algorithm_ptr->getPackageName();
    load_process_msg.request.executable = algorithm_ptr->getExecutable();

    TEMOTO_INFO("%s Found a suitable local algorithm: '%s', '%s', '%s', reliability %.3f"
              , prefix.c_str()
              , load_process_msg.request.action.c_str()
              , load_process_msg.request.package_name.c_str()
              , load_process_msg.request.executable.c_str()
              , algorithm_ptr->getReliability());

    // Request the Process Manager to load the algorithm
    try
    {
      resource_manager_.call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER
                                                  , process_manager::srv_name::SERVER
                                                  , load_process_msg
                                                  , rmp::FailureBehavior::UNLOAD_LINKED_RELOAD);
      algorithm_ptr->adjustReliability(1.0);
      advertiseAlgorithm(algorithm_ptr); // Let other managers know about the updated reliability
    }
    catch(error::ErrorStack& error_stack)
    {
      res.rmp = load_process_msg.response.rmp;
      algorithm_ptr->adjustReliability(0.0);
      advertiseAlgorithm(algorithm_ptr); // Let other managers know about the updated reliability
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("%s Call to Process Manager was sucessful.", prefix.c_str());

    // Fill out the response about which particular algorithm was chosen
    res.package_name = algorithm_ptr->getPackageName();
    res.topic = algorithm_ptr->getTopic();
    res.executable = algorithm_ptr->getExecutable();
    res.rmp = load_process_msg.response.rmp;

    // Add the allocated algorithm to the respective structure
    allocated_algorithms_.emplace(res.rmp.resource_id, algorithm_ptr);
    return;
  }

  // try remote algorithms
  for (auto& ra : remote_algorithms_)
  {
    TEMOTO_INFO("%s Looking from: \n%s",prefix.c_str(), ra->toString().c_str());
  }

  algorithm_ptr = findAlgorithm(req.algorithm_type, req.package_name, req.executable, remote_algorithms_);
  if (algorithm_ptr)
  {
    // remote algorithm candidate was found, forward the request to the remote algorithm manager
    temoto_2::LoadAlgorithm load_algorithm_msg;
    load_algorithm_msg.request.algorithm_type = algorithm_ptr->getType();
    load_algorithm_msg.request.package_name = algorithm_ptr->getPackageName();
    load_algorithm_msg.request.executable = algorithm_ptr->getExecutable();

    TEMOTO_INFO("AlgorithmManager is forwarding request: '%s', '%s', '%s', reliability %.3f"
              , algorithm_ptr->getType().c_str()
              , algorithm_ptr->getPackageName().c_str()
              , algorithm_ptr->getExecutable().c_str()
              , algorithm_ptr->getReliability());

    // Call the remote algorithm manager
    try
    {
      resource_manager_.call<temoto_2::LoadAlgorithm>(
          algorithm_manager::srv_name::MANAGER, algorithm_manager::srv_name::SERVER,
          load_algorithm_msg, rmp::FailureBehavior::UNLOAD_LINKED_RELOAD, algorithm_ptr->getTemotoNamespace());
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("%s Call to remote AlgorithmManager was sucessful.", prefix.c_str());

    res = load_algorithm_msg.response;
    allocated_algorithms_.emplace(res.rmp.resource_id, algorithm_ptr);
    return;
  }

  // No suitable local nor remote algorithm was found
  throw CREATE_ERROR(error::Code::ALGORITHM_NOT_FOUND, "AlgorithmManager did not find a suitable algorithm.");
}

/*
 * Unload algorithm callback
 */
void AlgorithmManager::unloadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req
                                       , temoto_2::LoadAlgorithm::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s received a request to stop algorithm with id '%ld'"
             , prefix.c_str()
             , res.rmp.resource_id);

  allocated_algorithms_.erase(res.rmp.resource_id);
  return;
}

/*
 * Find algorithm
 */
AlgorithmInfoPtr AlgorithmManager::findAlgorithm(std::string type
                                               , std::string package_name
                                               , std::string executable
                                               , const AlgorithmInfoPtrs& algorithm_infos)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  // Local list of devices that follow the requirements
  std::vector<AlgorithmInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(algorithm_infos.begin()
                       , algorithm_infos.end()
                       , std::back_inserter(candidates)
                       , [&](const AlgorithmInfoPtr& s)
                         {
                           return s->getType() == type;
                         });
  
  // The requested type of algorithm is not available
  if (candidates.empty())
  {
    return NULL;
  }
 
  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (package_name != "")
  {
    it_end = std::remove_if(candidates.begin()
                          , candidates.end()
                          , [&](AlgorithmInfoPtr s)
                            {
                              return s->getPackageName() != package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (executable != "")
  {
    it_end = std::remove_if(candidates.begin()
                          , it_end
                          , [&](AlgorithmInfoPtr s)
                            {
                              return s->getExecutable() != executable;
                            });
  }

  // Sort remaining candidates based on their reliability.
  std::sort(candidates.begin()
          , it_end
          , [](AlgorithmInfoPtr& s1, AlgorithmInfoPtr& s2)
            {
              return s1->getReliability() > s2->getReliability();
            });

  if (candidates.begin() == it_end)
  {
    // Algorithm with the requested criteria was not found.
    return NULL;
  }

  // Return the first algorithm of the requested type.
  return candidates.front();
}

/*
 * Parse algorithms
 */
AlgorithmInfoPtrs AlgorithmManager::parseAlgorithms(const YAML::Node& config)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  std::vector<AlgorithmInfoPtr> algorithms;

  //  TEMOTO_INFO("%s CONFIG NODE:%d %s", prefix.c_str(), config.Type(), Dump(config).c_str());
  if (!config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("%s Unable to parse 'algorithms' key from config.", prefix.c_str());
    return algorithms;
  }

  YAML::Node algorithms_node = config["Algorithms"];

  // Node throws
  TEMOTO_INFO("%s algorithms NODE:%d", prefix.c_str(), algorithms_node.Type());

  if (!algorithms_node.IsSequence())
  {
    TEMOTO_WARN("%s The given config does not contain sequence of algorithms.", prefix.c_str());
    // TODO Throw
    return algorithms;
  }

  TEMOTO_INFO("%s Parsing %lu algorithms.", prefix.c_str(), algorithms_node.size());

  // go over each algorithm node in the sequence
  for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("%s Unable to parse the algorithm. Parameters in YAML have to be specified in "
                 "key-value pairs.",
                 prefix.c_str());
      continue;
    }

    try
    {
      AlgorithmInfo algorithm = node_it->as<AlgorithmInfo>();
      if (std::count_if(algorithms.begin()
                      , algorithms.end()
                      , [&](const AlgorithmInfoPtr& s)
                        { return *s == algorithm; }) == 0)
      {
        // OK, this is unique algorithm information, add it to the algorithms vector.
        algorithms.emplace_back(std::make_shared<AlgorithmInfo>(algorithm));
      }
      else
      {
        TEMOTO_WARN("%s Ignoring duplicate of algorithm '%s'.", prefix.c_str(),
                    algorithm.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<AlgorithmInfo> e)
    {
      TEMOTO_WARN("%s Failed to parse AlgorithmInfo from config.", prefix.c_str());
      continue;
    }
  }
  return algorithms;
}

}  // algorithm_manager namespace
