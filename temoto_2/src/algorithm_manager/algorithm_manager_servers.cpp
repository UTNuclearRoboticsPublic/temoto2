#include "ros/package.h"
#include "algorithm_manager/algorithm_manager_servers.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace algorithm_manager
{
AlgorithmManagerServers::AlgorithmManagerServers(BaseSubsystem *b, AlgorithmInfoRegistry *sid)
  : BaseSubsystem(*b, __func__)
  , sid_(sid)
  , resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<temoto_2::LoadAlgorithm>( srv_name::SERVER
                                                   , &AlgorithmManagerServers::startAlgorithmCb
                                                   , &AlgorithmManagerServers::stopAlgorithmCb);
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
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_algorithms_.find(srv.request.resource_id);
    if (it != allocated_algorithms_.end())
    {
      if(it->second.isLocal())
      {
        TEMOTO_WARN("Local algorithm failure detected, adjusting reliability.");
        it->second.adjustReliability(0.0);
        sid_->updateLocalAlgorithm(it->second);
      }
      else
      {
        TEMOTO_WARN("Remote algorithm failure detected, doing nothing (algorithm will be updated via synchronizer).");
      }
    }
  }
}

// TODO: rename "startAlgorithmCb" to "loadAlgorithmCb"
void AlgorithmManagerServers::startAlgorithmCb( temoto_2::LoadAlgorithm::Request& req
                                        , temoto_2::LoadAlgorithm::Response& res)
{
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a algorithm: \n" << req << std::endl);

  // Try to find suitable candidate from local algorithms
  AlgorithmInfo si;
  if (sid_->findLocalAlgorithm(req, si))
  {
    // Try to run the algorithm via local Resource Manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = si.getPackageName();
    load_process_msg.request.executable = si.getExecutable();

    // Check if any particular topic types were requested
    if (!req.output_topics.empty())
    {
      // Remap the output topics if requested
      for (auto& req_topic : req.output_topics)
      {
        // And return the input topics via response
        diagnostic_msgs::KeyValue res_output_topic;
        res_output_topic.key = req_topic.key;
        std::string default_topic = si.getOutputTopic(req_topic.key);

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
    }
    else
    {
      TopicContainer output_topics;
      output_topics.setOutputTopics(si.getOutputTopics());

      // Translate all topics to absolute
      res.output_topics.clear();
      for (const auto& output_topic : si.getOutputTopics())
      {
        diagnostic_msgs::KeyValue topic_msg;
        topic_msg.key = output_topic.first;
        topic_msg.value = common::getAbsolutePath(output_topic.second);
        res.output_topics.push_back(topic_msg);
      }
    }
    TEMOTO_INFO( "AlgorithmManagerServers found a suitable local algorithm: '%s', '%s', '%s', reliability %.3f"
               , load_process_msg.request.action.c_str()
               , load_process_msg.request.package_name.c_str()
               , load_process_msg.request.executable.c_str(), si.getReliability());

    try
    {
      resource_manager_.call<temoto_2::LoadProcess>( process_manager::srv_name::MANAGER
                                                   , process_manager::srv_name::SERVER
                                                   , load_process_msg
                                                   , rmp::FailureBehavior::NONE);

      TEMOTO_DEBUG("Call to ProcessManager was sucessful.");

      // Fill out the response about which particular algorithm was chosen
      res.package_name = si.getPackageName();
      res.executable = si.getExecutable();
      res.rmp = load_process_msg.response.rmp;

      si.adjustReliability(1.0);
      sid_->updateLocalAlgorithm(si);
    }
    catch(error::ErrorStack& error_stack)
    { 
      if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
      {
        si.adjustReliability(0.0);
        sid_->updateLocalAlgorithm(si);
      }
      throw FORWARD_ERROR(error_stack);
    }

    allocated_algorithms_.emplace(res.rmp.resource_id, si);

    return;
  }

  // try remote algorithms
//  for (auto& rs : remote_algorithms_)
//  {
//    TEMOTO_INFO("Looking from: \n%s", rs->toString().c_str());
//  }

  if (sid_->findRemoteAlgorithm(req, si))
  {
    // remote algorithm candidate was found, forward the request to the remote algorithm manager
    temoto_2::LoadAlgorithm load_algorithm_msg;
    load_algorithm_msg.request.algorithm_type = si.getType();
    load_algorithm_msg.request.package_name = si.getPackageName();
    load_algorithm_msg.request.executable = si.getExecutable();
    load_algorithm_msg.request.output_topics = req.output_topics;

    TEMOTO_INFO("Algorithm Manager is forwarding request: '%s', '%s', '%s', reliability %.3f",
                si.getType().c_str(), si.getPackageName().c_str(),
                si.getExecutable().c_str(), si.getReliability());

    try
    {
      resource_manager_.call<temoto_2::LoadAlgorithm>( algorithm_manager::srv_name::MANAGER
                                                  , algorithm_manager::srv_name::SERVER
                                                  , load_algorithm_msg
                                                  , rmp::FailureBehavior::NONE
                                                  , si.getTemotoNamespace());

      TEMOTO_DEBUG("Call to remote AlgorithmManagerServers was sucessful.");
      res = load_algorithm_msg.response;
      allocated_algorithms_.emplace(res.rmp.resource_id, si);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    return;
  }
  else
  {
    // no suitable local nor remote algorithm was found
    throw CREATE_ERROR(error::Code::SENSOR_NOT_FOUND, "AlgorithmManagerServers did not find a suitable algorithm.");
  }
}

// TODO: rename "stopAlgorithmCb" to "unloadAlgorithmCb"
void AlgorithmManagerServers::stopAlgorithmCb(temoto_2::LoadAlgorithm::Request& req,
                                 temoto_2::LoadAlgorithm::Response& res)
{
  TEMOTO_DEBUG("received a request to stop algorithm with id '%ld'", res.rmp.resource_id);
  allocated_algorithms_.erase(res.rmp.resource_id);
  return;
}

}  // algorithm_manager namespace
