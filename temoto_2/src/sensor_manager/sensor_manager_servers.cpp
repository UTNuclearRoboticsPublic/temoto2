#include "ros/package.h"
#include "sensor_manager/sensor_manager_servers.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace sensor_manager
{
SensorManagerServers::SensorManagerServers(BaseSubsystem *b, SensorInfoDatabase *sid)
  : BaseSubsystem(*b, __func__)
  , sid_(sid)
  , resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>( srv_name::SERVER
                                                   , &SensorManagerServers::startSensorCb
                                                   , &SensorManagerServers::stopSensorCb);
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
        sid_->updateLocalSensor(it->second);
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
  for (const auto& sensor : sid_->getLocalSensors())
  {
    if (sensor.getType() == req.type)
    {
      res.list.push_back(sensor.getName());
    }
  }

  return true;
}

// TODO: rename "startSensorCb" to "loadSensorCb"
void SensorManagerServers::startSensorCb( temoto_2::LoadSensor::Request& req
                                        , temoto_2::LoadSensor::Response& res)
{
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a sensor: \n" << req << std::endl);

  // Try to find suitable candidate from local sensors
  SensorInfo si;
  if (sid_->findLocalSensor(req, si))
  {
    // Try to run the sensor via local Resource Manager
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
    TEMOTO_INFO( "SensorManagerServers found a suitable local sensor: '%s', '%s', '%s', reliability %.3f"
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

      // Fill out the response about which particular sensor was chosen
      res.package_name = si.getPackageName();
      res.executable = si.getExecutable();
      res.rmp = load_process_msg.response.rmp;

      si.adjustReliability(1.0);
      sid_->updateLocalSensor(si);
    }
    catch(error::ErrorStack& error_stack)
    { 
      if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
      {
        si.adjustReliability(0.0);
        sid_->updateLocalSensor(si);
      }
      throw FORWARD_ERROR(error_stack);
    }

    allocated_sensors_.emplace(res.rmp.resource_id, si);

    return;
  }

  // try remote sensors
//  for (auto& rs : remote_sensors_)
//  {
//    TEMOTO_INFO("Looking from: \n%s", rs->toString().c_str());
//  }

  if (sid_->findRemoteSensor(req, si))
  {
    // remote sensor candidate was found, forward the request to the remote sensor manager
    temoto_2::LoadSensor load_sensor_msg;
    load_sensor_msg.request.sensor_type = si.getType();
    load_sensor_msg.request.package_name = si.getPackageName();
    load_sensor_msg.request.executable = si.getExecutable();
    load_sensor_msg.request.output_topics = req.output_topics;

    TEMOTO_INFO("Sensor Manager is forwarding request: '%s', '%s', '%s', reliability %.3f",
                si.getType().c_str(), si.getPackageName().c_str(),
                si.getExecutable().c_str(), si.getReliability());

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
  else
  {
    // no suitable local nor remote sensor was found
    throw CREATE_ERROR(error::Code::SENSOR_NOT_FOUND, "SensorManagerServers did not find a suitable sensor.");
  }
}

// TODO: rename "stopSensorCb" to "unloadSensorCb"
void SensorManagerServers::stopSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  TEMOTO_DEBUG("received a request to stop sensor with id '%ld'", res.rmp.resource_id);
  allocated_sensors_.erase(res.rmp.resource_id);
  return;
}

}  // sensor_manager namespace
