#include "ros/package.h"
#include "sensor_manager/sensor_manager_servers.h"

#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <regex>

namespace sensor_manager
{
SensorManagerServers::SensorManagerServers(BaseSubsystem *b, SensorInfoRegistry *sir)
  : BaseSubsystem(*b, __func__)
  , sir_(sir)
  , resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>( srv_name::SERVER
                                                   , &SensorManagerServers::loadSensorCb
                                                   , &SensorManagerServers::unloadSensorCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&SensorManagerServers::statusCb);


  TEMOTO_INFO("Sensor manager is ready.");
}

SensorManagerServers::~SensorManagerServers()
{
}

void SensorManagerServers::statusCb(temoto_2::ResourceStatus& srv)
{

  TEMOTO_DEBUG("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_sensors_.find(srv.request.resource_id);
    if (it != allocated_sensors_.end())
    {
      if(it->second.isLocal())
      {
        TEMOTO_WARN("Local sensor failure detected, adjusting reliability.");
        it->second.adjustReliability(0.0);
        sir_->updateLocalSensor(it->second);
      }
      else
      {
        TEMOTO_WARN("Remote sensor failure detected, doing nothing (sensor will be updated via synchronizer).");
      }
    }
  }
}

bool SensorManagerServers::listDevicesCb( temoto_2::ListDevices::Request& req
                                        , temoto_2::ListDevices::Response& res)
{
  // std::vector <std::string> devList;

  // Find the devices with the required type
  for (const auto& sensor : sir_->getLocalSensors())
  {
    if (sensor.getType() == req.type)
    {
      res.list.push_back(sensor.getName());
    }
  }

  return true;
}

// TODO: rename "loadSensorCb" to "loadSensorCb"
void SensorManagerServers::loadSensorCb( temoto_2::LoadSensor::Request& req
                                        , temoto_2::LoadSensor::Response& res)
{
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a sensor: \n" << req << std::endl);

  // Try to find suitable candidate from local sensors
  std::vector<SensorInfo> l_sis;
  std::vector<SensorInfo> r_sis;

  bool got_local_sensors = sir_->findLocalSensors(req, l_sis);
  bool got_remote_sensors = sir_->findRemoteSensors(req, r_sis);

  // Find the most reliable global sensor
  bool prefer_remote = false;
  if (got_local_sensors && got_remote_sensors)
  {
    if (l_sis.at(0).getReliability() < r_sis.at(0).getReliability())
    {
      prefer_remote = true;
    }
  }

  if (got_local_sensors && !prefer_remote)
  {
    // Loop over suitable sensors
    for (SensorInfo& si : l_sis)
    {
      // Try to run the sensor via local Resource Manager
      temoto_2::LoadProcess load_process_msg;
      load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
      load_process_msg.request.package_name = si.getPackageName();
      load_process_msg.request.executable = si.getExecutable();

      // Remap the input topics if requested
      processTopics(req.input_topics, res.input_topics, load_process_msg, si, "in");

      // Remap the output topics if requested
      processTopics(req.output_topics, res.output_topics, load_process_msg, si, "out");

      TEMOTO_INFO( "SensorManagerServers found a suitable local sensor: '%s', '%s', '%s', reliability %.3f"
                 , load_process_msg.request.action.c_str()
                 , load_process_msg.request.package_name.c_str()
                 , load_process_msg.request.executable.c_str()
                 , si.getReliability());

      try
      {
        resource_manager_.call<temoto_2::LoadProcess>( process_manager::srv_name::MANAGER
                                                     , process_manager::srv_name::SERVER
                                                     , load_process_msg
                                                     , rmp::FailureBehavior::NONE);

        TEMOTO_DEBUG("Call to ProcessManager was sucessful.");

        // Fill out the response about which particular sensor was chosen
        res.package_name = si.getPackageName();
        res.executable = si.getExecutable();
        res.rmp = load_process_msg.response.rmp;

        si.adjustReliability(1.0);
        sir_->updateLocalSensor(si);
        allocated_sensors_.emplace(res.rmp.resource_id, si);

        return;
      }
      catch(error::ErrorStack& error_stack)
      {
        if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
        {
          si.adjustReliability(0.0);
          sir_->updateLocalSensor(si);
        }
        SEND_ERROR(error_stack);
      }
    }
  }

  // try remote sensors
//  for (auto& rs : remote_sensors_)
//  {
//    TEMOTO_INFO("Looking from: \n%s", rs->toString().c_str());
//  }

  if (got_remote_sensors)
  {
    // Loop over suitable sensors
    for (SensorInfo& si : r_sis)
    {
      // remote sensor candidate was found, forward the request to the remote sensor manager
      temoto_2::LoadSensor load_sensor_msg;
      load_sensor_msg.request.sensor_type = si.getType();
      load_sensor_msg.request.package_name = si.getPackageName();
      load_sensor_msg.request.executable = si.getExecutable();
      load_sensor_msg.request.input_topics = req.input_topics;
      load_sensor_msg.request.output_topics = req.output_topics;

      TEMOTO_INFO( "Sensor Manager is forwarding request: '%s', '%s', '%s', reliability %.3f"
                 , si.getType().c_str()
                 , si.getPackageName().c_str()
                 , si.getExecutable().c_str()
                 , si.getReliability());

      try
      {
        resource_manager_.call<temoto_2::LoadSensor>( sensor_manager::srv_name::MANAGER
                                                    , sensor_manager::srv_name::SERVER
                                                    , load_sensor_msg
                                                    , rmp::FailureBehavior::NONE
                                                    , si.getTemotoNamespace());

        TEMOTO_DEBUG("Call to remote SensorManagerServers was sucessful.");
        res = load_sensor_msg.response;
        allocated_sensors_.emplace(res.rmp.resource_id, si);
      }
      catch(error::ErrorStack& error_stack)
      {
        throw FORWARD_ERROR(error_stack);
      }
      return;
    }
  }
  else
  {
    // no suitable local nor remote sensor was found
    throw CREATE_ERROR(error::Code::SENSOR_NOT_FOUND, "SensorManagerServers did not find a suitable sensor.");
  }
}

// TODO: rename "unloadSensorCb" to "unloadSensorCb"
void SensorManagerServers::unloadSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  TEMOTO_DEBUG("received a request to stop sensor with id '%ld'", res.rmp.resource_id);
  allocated_sensors_.erase(res.rmp.resource_id);
  return;
}

void SensorManagerServers::processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
                                        , std::vector<diagnostic_msgs::KeyValue>& res_topics
                                        , temoto_2::LoadProcess& load_process_msg
                                        , SensorInfo& sensor_info
                                        , std::string direction)
{
  /*
   * Find out it this is a launch file or not. Remapping is different
   * for executable types (launch files or executables)
   */
  bool isLaunchFile;
  std::regex rx(".*\\.launch$");
  isLaunchFile = std::regex_match(sensor_info.getExecutable(), rx);

  // Work with input or output topics
  std::vector<StringPair> sensor_info_topics;

  if (direction == "in")
  {
    sensor_info_topics = sensor_info.getInputTopics();
  }
  else if (direction == "out")
  {
    sensor_info_topics = sensor_info.getOutputTopics();
  }

  // If no topics were requested, then return a list of all topics this sensor publishes
  if (req_topics.empty())
  {
    for (const auto& output_topic : sensor_info_topics)
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

    if (direction == "in")
    {
      default_topic = sensor_info.getInputTopic(req_topic.key);
    }
    else if (direction == "out")
    {
      default_topic = sensor_info.getOutputTopic(req_topic.key);
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

}  // sensor_manager namespace
