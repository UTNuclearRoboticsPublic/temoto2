/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                            TODO:
 *
 * 1) Currently algorithm info requires that all topic types are
 *    unique. For example there cannot be two camera topics that
 *    have the same type ...
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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
  : BaseSubsystem("algorithm_manager", error::Subsystem::ALGORITHM_MANAGER, __func__)
  , resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &AlgorithmManager::syncCb, this)
{

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
      TEMOTO_DEBUG("Added algorithm: '%s'.", s->getName().c_str());
    }

    // notify other managers about our algorithms
    advertiseLocalAlgorithms();
  }
  else
  {
    TEMOTO_WARN("Failed to read '%s'. Verify that the file exists and the sequence of algorithms "
                "is listed under 'Algorithms' node.",
                yaml_filename.c_str());
  }
  ///////////////////////////////////////////////////////////////////////////////////////

  TEMOTO_INFO_STREAM("Algorithm manager is ready.");
}

/*
 * Status callback
 */
void AlgorithmManager::statusCb(temoto_2::ResourceStatus& srv)
{
  TEMOTO_DEBUG("Received a status message");

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
  try
  {
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
//      std::cout << YAML::Dump(config) << std::endl;
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
          TEMOTO_DEBUG("Updating remote algorithm '%s' at '%s'.",
              algorithm->getName().c_str(), algorithm->getTemotoNamespace().c_str());

          // Update by owerwriting the old entry
          *it = algorithm;
        }

        // Add new algorithm
        else
        {
          TEMOTO_DEBUG("Adding remote algorithm '%s' at '%s'.",
              algorithm->getName().c_str(), algorithm->getTemotoNamespace().c_str());

          remote_algorithms_.push_back(algorithm);
        }
      }
    }
  }
  // Catch temoto errors
  catch(error::ErrorStack error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }

  // Catch yaml errors
  catch (YAML::InvalidNode e)
  {
    throw CREATE_ERROR(error::Code::YAML_ERROR, e.what());
  }

  // Catch all other errors
  catch(...)
  {
    throw CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception.");
  }
}

/*
 * Advertise local algorithm
 */
void AlgorithmManager::advertiseAlgorithm(AlgorithmInfoPtr algorithm_ptr)
{
    YAML::Node config;
    config["Algorithms"].push_back(*algorithm_ptr);
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
 * \TODO: Make sure all the topics returned from this function are ABSOLUTE!
 */
void AlgorithmManager::loadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req
                                     , temoto_2::LoadAlgorithm::Response& res)
{
  TEMOTO_DEBUG("received a request to load '%s': '%s', '%s'"
             , req.algorithm_type.c_str()
             , req.package_name.c_str()
             , req.executable.c_str());

//  TEMOTO_DEBUG_STREAM("\n IN MORE DETAIL: \n" << req << "\n");

  // Try to find suitable candidate from local algorithms
  auto algorithm_ptr = findAlgorithm(req, local_algorithms_);
  if (algorithm_ptr)
  {
    // Local algorithm found, prepare the message and make a call to the local resource manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = algorithm_ptr->getPackageName();
    load_process_msg.request.executable = algorithm_ptr->getExecutable();

    // Remap the input topics if requested
    for (auto& req_topic : req.input_topics)
    {
      // And return the input topics via response
      diagnostic_msgs::KeyValue res_input_topic;
      res_input_topic.key = req_topic.key;
      std::string default_topic = algorithm_ptr->getInputTopic(req_topic.key);

      if (req_topic.value != "")
      {
        res_input_topic.value = common::getAbsolutePath(req_topic.value);
        std::string remap_arg = default_topic + ":=" + req_topic.value;
        load_process_msg.request.args += remap_arg + " ";
      }
      else
      {
        res_input_topic.value = common::getAbsolutePath(default_topic);
      }

      // Add the topic to the response message
      res.input_topics.push_back(res_input_topic);
    }

    // Remap the output topics if requested
    for (auto& req_topic : req.output_topics)
    {
      // And return the input topics via response
      diagnostic_msgs::KeyValue res_output_topic;
      res_output_topic.key = req_topic.key;
      std::string default_topic = algorithm_ptr->getOutputTopic(req_topic.key);

      if (req_topic.value != "")
      {
        res_output_topic.value = common::getAbsolutePath(req_topic.value);
        std::string remap_arg = default_topic + ":=" + req_topic.value;
        load_process_msg.request.args += remap_arg + " ";
      }
      else
      {
        res_output_topic.value = common::getAbsolutePath(default_topic);
      }

      // Add the topic to the response message
      res.output_topics.push_back(res_output_topic);
    }

    TEMOTO_DEBUG("Sending arguments: '%s'.", load_process_msg.request.args.c_str());

    TEMOTO_INFO("Found a suitable local algorithm: '%s', '%s', '%s', reliability %.3f"
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
                                                  , rmp::FailureBehavior::NONE);
      algorithm_ptr->adjustReliability(1.0);

      // Let other managers know about the updated reliability
      advertiseAlgorithm(algorithm_ptr);
    }
    catch(error::ErrorStack& error_stack)
    {
      if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
      {
        algorithm_ptr->adjustReliability(0.0);
        advertiseAlgorithm(algorithm_ptr); // Let other managers know about the updated reliability
      }

      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("Call to Process Manager was sucessful.");

    // Fill out the response about which particular algorithm was chosen
    res.package_name = algorithm_ptr->getPackageName();
    res.executable = algorithm_ptr->getExecutable();
    res.rmp = load_process_msg.response.rmp;

    // Add the allocated algorithm to the respective structure
    allocated_algorithms_.emplace(res.rmp.resource_id, algorithm_ptr);
    return;
  }

  TEMOTO_DEBUG("Could not find the requested algorithm form local algorithms. "
               "Trying to find the algorithm from known remote algorithms");

  /*
   * If no local algorithms were found, then look them from the list
   * of known remote algorithms
   */
  for (auto& ra : remote_algorithms_)
  {
    TEMOTO_INFO("Looking from: \n%s", ra->toString().c_str());
  }

  algorithm_ptr = findAlgorithm(req, remote_algorithms_);
  if (algorithm_ptr)
  {
    // remote algorithm candidate was found, forward the request to the remote algorithm manager
    temoto_2::LoadAlgorithm load_algorithm_msg;
    load_algorithm_msg.request.algorithm_type = algorithm_ptr->getType();
    load_algorithm_msg.request.package_name = algorithm_ptr->getPackageName();
    load_algorithm_msg.request.executable = algorithm_ptr->getExecutable();
    load_algorithm_msg.request.input_topics = req.input_topics;
    load_algorithm_msg.request.output_topics = req.output_topics;

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
          load_algorithm_msg, rmp::FailureBehavior::NONE, algorithm_ptr->getTemotoNamespace());
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("Call to remote AlgorithmManager was sucessful.");

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
  TEMOTO_DEBUG("received a request to stop algorithm with id '%ld'", res.rmp.resource_id);

  allocated_algorithms_.erase(res.rmp.resource_id);
  return;
}

/*
 * Find algorithm
 */
AlgorithmInfoPtr AlgorithmManager::findAlgorithm(temoto_2::LoadAlgorithm::Request req
                                               , const AlgorithmInfoPtrs& algorithm_infos)
{
  // Local list of devices that follow the requirements
  std::vector<AlgorithmInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(algorithm_infos.begin()
                       , algorithm_infos.end()
                       , std::back_inserter(candidates)
                       , [&](const AlgorithmInfoPtr& s)
                         {
                           return s->getType() == req.algorithm_type;
                         });
  
  // The requested type of algorithm is not available
  if (candidates.empty())
  {
    return NULL;
  }

  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin()
                          , candidates.end()
                          , [&](AlgorithmInfoPtr s)
                            {
                              return s->getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin()
                          , it_end
                          , [&](AlgorithmInfoPtr s)
                            {
                              return s->getExecutable() != req.executable;
                            });
  }

  // If input topics are specified ...
  if (!req.input_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                           [&](AlgorithmInfoPtr s)
                           {
                             if (s->getTopicsIn().size() != req.input_topics.size())
                               return true;

                             // Make a copy of the input topics
                             std::vector<StringPair> input_topics_copy = s->getTopicsIn();

                             // Start looking for the requested topic types
                             for (auto& topic : req.input_topics)
                             {
                               bool found = false;
                               for (auto it=input_topics_copy.begin(); it != input_topics_copy.end(); it++)
                               {
                                 // If the topic was found then remove it from the copy list
                                 if (topic.key == it->first)
                                 {
                                   found = true;
                                   input_topics_copy.erase(it);
                                   break;
                                 }
                               }

                               // If this topic type was not found then return with false
                               if (!found)
                               {
                                 return true;
                               }
                             }

                             return false;
                           });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](AlgorithmInfoPtr s)
                            {
                              if (s->getTopicsOut().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s->getTopicsOut();

                              // Start looking for the requested topic types
                              for (auto& topic : req.output_topics)
                              {
                                bool found = false;
                                for (auto it=output_topics_copy.begin(); it != output_topics_copy.end(); it++)
                                {
                                  // If the topic was found then remove it from the copy list
                                  if (topic.key == it->first)
                                  {
                                    found = true;
                                    output_topics_copy.erase(it);
                                    break;
                                  }
                                }

                                // If this topic type was not found then return with false
                                if (!found)
                                {
                                  return true;
                                }
                              }

                              return false;
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
    TEMOTO_ERROR_STREAM("no candidates found");

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
  std::vector<AlgorithmInfoPtr> algorithms;

  if (!config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("Unable to parse 'algorithms' key from config.");
    return algorithms;
  }

  YAML::Node algorithms_node = config["Algorithms"];

  // Node throws
  TEMOTO_INFO("Algorithms NODE:%d", algorithms_node.Type());

  if (!algorithms_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of algorithms.");
    // TODO Throw
    return algorithms;
  }

  TEMOTO_INFO("Parsing %lu algorithms.", algorithms_node.size());

  // go over each algorithm node in the sequence
  for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("Unable to parse the algorithm. Parameters in YAML have to be specified in "
                   "key-value pairs.");
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
        TEMOTO_WARN("Ignoring duplicate of algorithm '%s'.", algorithm.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<AlgorithmInfo> e)
    {
      TEMOTO_WARN("Failed to parse AlgorithmInfo from config: %s", e.what());
      continue;
    }
  }
  return algorithms;
}

}  // algorithm_manager namespace
