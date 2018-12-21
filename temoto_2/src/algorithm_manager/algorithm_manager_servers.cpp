#include "ros/package.h"
#include "algorithm_manager/algorithm_manager_servers.h"

#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <regex>

namespace algorithm_manager
{
AlgorithmManagerServers::AlgorithmManagerServers(temoto_core::BaseSubsystem *b, AlgorithmInfoRegistry *air)
  : temoto_core::BaseSubsystem(*b, __func__)
  , air_(air)
  , resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<temoto_2::LoadAlgorithm>( srv_name::SERVER
                                                   , &AlgorithmManagerServers::loadAlgorithmCb
                                                   , &AlgorithmManagerServers::unloadAlgorithmCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&AlgorithmManagerServers::statusCb);


  TEMOTO_INFO("Algorithm manager is ready.");
}

AlgorithmManagerServers::~AlgorithmManagerServers()
{
}

void AlgorithmManagerServers::statusCb(temoto_2::ResourceStatus& srv)
{

  TEMOTO_DEBUG("Received a status message.");

  // If local algorithm failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
  {
    auto it = allocated_algorithms_.find(srv.request.resource_id);
    if (it != allocated_algorithms_.end())
    {
      if(it->second.isLocal())
      {
        TEMOTO_WARN("Local algorithm failure detected, adjusting reliability.");
        it->second.adjustReliability(0.0);
        air_->updateLocalAlgorithm(it->second);
      }
      else
      {
        TEMOTO_WARN("Remote algorithm failure detected, doing nothing (algorithm will be updated via synchronizer).");
      }
    }
  }
}

void AlgorithmManagerServers::loadAlgorithmCb( temoto_2::LoadAlgorithm::Request& req
                                             , temoto_2::LoadAlgorithm::Response& res)
{
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a algorithm: \n" << req << std::endl);

  // Try to find suitable candidate from local algorithms
  std::vector<AlgorithmInfo> ais;
  if (air_->findLocalAlgorithms(req, ais))
  {
    // Loop over suitable algorithms
    for (AlgorithmInfo& ai : ais)
    {
      // Try to run the algorithm via local Resource Manager
      temoto_2::LoadProcess load_process_msg;
      load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
      load_process_msg.request.package_name = ai.getPackageName();
      load_process_msg.request.executable = ai.getExecutable();

      // Remap the input topics if requested
      processTopics(req.input_topics, res.input_topics, load_process_msg, ai, true);

      // Remap the output topics if requested
      processTopics(req.output_topics, res.output_topics, load_process_msg, ai, false);

      TEMOTO_INFO( "Found a suitable local algorithm: '%s', '%s', '%s', reliability %.3f"
                 , load_process_msg.request.action.c_str()
                 , load_process_msg.request.package_name.c_str()
                 , load_process_msg.request.executable.c_str()
                 , ai.getReliability());

      try
      {
        resource_manager_.call<temoto_2::LoadProcess>( process_manager::srv_name::MANAGER
                                                     , process_manager::srv_name::SERVER
                                                     , load_process_msg
                                                     , temoto_core::rmp::FailureBehavior::NONE);

        TEMOTO_DEBUG("Call to ProcessManager was sucessful.");

        // Fill out the response about which particular algorithm was chosen
        res.package_name = ai.getPackageName();
        res.executable = ai.getExecutable();
        res.rmp = load_process_msg.response.rmp;

        ai.adjustReliability(1.0);
        air_->updateLocalAlgorithm(ai);
        allocated_algorithms_.emplace(res.rmp.resource_id, ai);

        return;
      }
      catch(temoto_core::error::ErrorStack& error_stack)
      {
        if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
        {
          ai.adjustReliability(0.0);
          air_->updateLocalAlgorithm(ai);
        }
        // TODO: Create error branches
        SEND_ERROR(error_stack);
      }
    }
  }

  /*
   * try remote algorithms
   */
  if (air_->findRemoteAlgorithms(req, ais))
  {
    for (AlgorithmInfo& ai : ais)
    {
      // remote algorithm candidate was found, forward the request to the remote algorithm manager
      temoto_2::LoadAlgorithm load_algorithm_msg;
      load_algorithm_msg.request.algorithm_type = ai.getType();
      load_algorithm_msg.request.package_name = ai.getPackageName();
      load_algorithm_msg.request.executable = ai.getExecutable();
      load_algorithm_msg.request.input_topics = req.input_topics;
      load_algorithm_msg.request.output_topics = req.output_topics;

      TEMOTO_INFO( "Algorithm Manager is forwarding request: '%s', '%s', '%s', reliability %.3f"
                 , ai.getType().c_str()
                 , ai.getPackageName().c_str()
                 , ai.getExecutable().c_str()
                 , ai.getReliability());

      try
      {
        resource_manager_.call<temoto_2::LoadAlgorithm>( algorithm_manager::srv_name::MANAGER
                                                       , algorithm_manager::srv_name::SERVER
                                                       , load_algorithm_msg
                                                       , temoto_core::rmp::FailureBehavior::NONE
                                                       , ai.getTemotoNamespace());

        TEMOTO_DEBUG("Call to remote AlgorithmManagerServers was sucessful.");
        res = load_algorithm_msg.response;
        allocated_algorithms_.emplace(res.rmp.resource_id, ai);

        return;
      }
      catch(temoto_core::error::ErrorStack& error_stack)
      {
        // TODO: Create error branches
        SEND_ERROR(error_stack);
      }
    }
  }
  else
  {
    // no suitable local nor remote algorithm was found
    throw CREATE_ERROR(error::Code::ALGORITHM_NOT_FOUND, "Did not find a suitable algorithm.");
  }
}

// TODO: rename "unloadAlgorithmCb" to "unloadAlgorithmCb"
void AlgorithmManagerServers::unloadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req,
                                 temoto_2::LoadAlgorithm::Response& res)
{
  TEMOTO_DEBUG("received a request to stop algorithm with id '%ld'", res.rmp.resource_id);
  allocated_algorithms_.erase(res.rmp.resource_id);
  return;
}

void AlgorithmManagerServers::processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
                                           , std::vector<diagnostic_msgs::KeyValue>& res_topics
                                           , temoto_2::LoadProcess& load_process_msg
                                           , AlgorithmInfo& algorithm_info
                                           , bool inputTopics)
{
  /*
   * Find out it this is a launch file or not. Remapping is different
   * for executable types (launch files or executables)
   */
  bool isLaunchFile;
  std::regex rx(".*\\.launch$");
  isLaunchFile = std::regex_match(algorithm_info.getExecutable(), rx);

  // If no topics were requested, then return a list of all topics this algorithm publishes
  if (req_topics.empty())
  {
    for (const auto& output_topic : algorithm_info.getOutputTopics())
    {
      diagnostic_msgs::KeyValue topic_msg;
      topic_msg.key = output_topic.first;
      topic_msg.value = common::getAbsolutePath(output_topic.second);
      res_topics.push_back(topic_msg);
    }
    return;
  }

  // Remap the input topics if requested
  for (auto& req_topic : req_topics)
  {
    // And return the input topics via response
    diagnostic_msgs::KeyValue res_topic;
    res_topic.key = req_topic.key;
    std::string default_topic;

    if (inputTopics)
    {
      default_topic = algorithm_info.getInputTopic(req_topic.key);
    }
    else
    {
      default_topic = algorithm_info.getOutputTopic(req_topic.key);
    }

    if (req_topic.value != "")
    {
      res_topic.value = common::getAbsolutePath(req_topic.value);

      // Remap depending wether it is a launch file or excutable
      std::string remap_arg;

      if (isLaunchFile)
      {
        remap_arg = req_topic.key + ":=" + req_topic.value;
      }
      else
      {
        remap_arg = default_topic + ":=" + req_topic.value;
      }

      load_process_msg.request.args += remap_arg + " ";
    }
    else
    {
      res_topic.value = common::getAbsolutePath(default_topic);
    }

    // Add the topic to the response message
    res_topics.push_back(res_topic);
  }
}

}  // algorithm_manager namespace
