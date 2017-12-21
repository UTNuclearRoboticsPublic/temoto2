/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO:
 *          * CATCH ALL EXEPTIONS !!!
 *          * ADD A SERVICE PROCESSING QUEUEINSTEAD OF WAITING
 *          * MOST OF THE CODE IS COMPLETELY INITIAL AND
 *            HAS ONLY SINGLE FUNCTIONALITY, I.E. LAUNCHFILES
 *            ARE NOT SUPPORTED, ETC.
 *          * CHANGE THE "start_sensor_cb" to "start_sensor_cb"
 *          * KEEP TRACK ON WHAT SENSORS ARE CURRENTLY RUNNING
 *            (sensors, registered subscribers. if subs = 0 then
 *             shut the sensor down)
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/package.h"
#include "core/common.h"
#include "sensor_manager/sensor_manager.h"
#include "sensor_manager/sensor_manager_errors.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace sensor_manager
{
SensorManager::SensorManager()
  : resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &SensorManager::syncCb, this)
{
  class_name_ = __func__;
  subsystem_name_ = "sensor_manager";
  subsystem_code_ = error::Subsystem::SENSOR_MANAGER;
  log_group_ = "sensor_manager";
  error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);

  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, "");

  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>(srv_name::SERVER, &SensorManager::startSensorCb,
                                                    &SensorManager::stopSensorCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&SensorManager::statusCb);

  config_syncer_.requestRemoteConfigs();

  // Read the sensors for this manager. 
  std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                              common::getTemotoNamespace() + ".yaml";
  std::ifstream in(yaml_filename);
  YAML::Node config = YAML::Load(in);
  if (config["Sensors"])
  {
    local_sensors_ = parseSensors(config);
    for (auto& s : local_sensors_)
    {
      TEMOTO_DEBUG("%s Added sensor: '%s'.", prefix.c_str(), s->getName().c_str());
    }
    // notify other managers about our sensors
    advertiseLocalSensors();
  }
  else
  {
    TEMOTO_WARN("%s Failed to read '%s'. Verify that the file exists and the sequence of sensors "
                "is listed under 'Sensors' node.",
                prefix.c_str(), yaml_filename.c_str());
  }


  TEMOTO_INFO("Sensor manager is ready.");
}

SensorManager::~SensorManager()
{
}

void SensorManager::statusCb(temoto_2::ResourceStatus& srv)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s Status received.", prefix.c_str());
  // adjust package reliability when someone reported that it has failed.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_sensors_.find(srv.request.resource_id);
    if (it != allocated_sensors_.end())
    {
      TEMOTO_WARN("Sensor failure detected, adjusting reliability.");
      it->second->adjustReliability(0.0);
      YAML::Node config;
      config["Sensors"].push_back(*it->second);

      PayloadType payload;
      payload.data = Dump(config);
      config_syncer_.advertise(payload);
    }
  }
}

bool SensorManager::listDevicesCb(temoto_2::ListDevices::Request& req,
                                  temoto_2::ListDevices::Response& res)
{
  // std::vector <std::string> devList;

  // Find the devices with the required type
  for (auto& entry : local_sensors_)
  {
    if (entry->getType() == req.type)
    {
      res.list.push_back(entry->getName());
    }
  }

  return true;
}

void SensorManager::syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseLocalSensors();
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(payload.data);
    std::vector<SensorInfoPtr> sensors = parseSensors(config);

    // TODO: Hold remote stuff in a map or something keyed by namespace
    // TODO: Temoto namespace can (doesn't have to) be contained in config
    for (auto& s : sensors)
    {
      s->setTemotoNamespace(msg.temoto_namespace);
    }

    for (auto& sensor : sensors)
    {
      // Check if sensor has to be added or updated
      auto it = std::find_if(remote_sensors_.begin(), remote_sensors_.end(),
          [&](const SensorInfoPtr& rs) { return *rs == *sensor; });
      if (it != remote_sensors_.end())
      {
        TEMOTO_DEBUG("%s Updating remote sensor '%s' at '%s'.", prefix.c_str(),
            sensor->getName().c_str(), sensor->getTemotoNamespace().c_str());
        *it = sensor; // overwrite found sensor
      }
      else
      {
        TEMOTO_DEBUG("%s Adding remote sensor '%s' at '%s'.", prefix.c_str(),
            sensor->getName().c_str(), sensor->getTemotoNamespace().c_str());
        remote_sensors_.push_back(sensor);
      }
    }
  }
}

void SensorManager::advertiseLocalSensors()
{
    // publish all local sensors
    YAML::Node config;
    for(auto& s : local_sensors_) 
    {
        config["Sensors"].push_back(*s);
    }
    
    // send to other managers if there is anything to send
    if(config.size())
    {
      PayloadType payload;
      payload.data = Dump(config);
      config_syncer_.advertise(payload);
    }
}


// TODO: rename "startSensorCb" to "loadSensorCb"
void SensorManager::startSensorCb(temoto_2::LoadSensor::Request& req,
                                  temoto_2::LoadSensor::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s received a request to start '%s': '%s', '%s'", prefix.c_str(),
               req.sensor_type.c_str(), req.package_name.c_str(), req.executable.c_str());

  TEMOTO_DEBUG_STREAM("\n IN MORE DETAIL: \n" << req << "\n");

  // Try to find suitable candidate from local sensors
  auto sensor_ptr = findSensor(req, local_sensors_);
  if (sensor_ptr)
  {
    // local sensor found, make a call to the local resource manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = sensor_ptr->getPackageName();
    load_process_msg.request.executable = sensor_ptr->getExecutable();

    // Remap the output topics if requested
    for (auto& req_topic : req.output_topics)
    {
      // And return the input topics via response
      diagnostic_msgs::KeyValue res_output_topic;
      res_output_topic.key = req_topic.key;
      std::string default_topic = sensor_ptr->getOutputTopic(req_topic.key);

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

    TEMOTO_INFO("SensorManager found a suitable local sensor: '%s', '%s', '%s', reliability %.3f",
                load_process_msg.request.action.c_str(),
                load_process_msg.request.package_name.c_str(),
                load_process_msg.request.executable.c_str(), sensor_ptr->getReliability());

    if (!resource_manager_.call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER,
                                                      process_manager::srv_name::SERVER,
                                                      load_process_msg))
    {
      // Respond with an error message
      res.rmp.code = 1;
      res.rmp.message = "Failed to call the ProcessManager.";
      res.rmp.errorStack = error_handler_.createAndReturn(ErrorCode::SERVICE_REQ_FAIL
                                                        , prefix
                                                        , res.rmp.message);
      return;
    }

    // Increase or decrease the reliability depending on the return code
    if (res.rmp.code == 0)
    {
      sensor_ptr->adjustReliability(1.0);

      // Fill out the response about which particular sensor was chosen
      res.package_name = sensor_ptr->getPackageName();
      res.executable = sensor_ptr->getExecutable();
      res.topic = sensor_ptr->getTopic(); // TODO: DEPRECATED !!!
      res.rmp = load_process_msg.response.rmp;

      // Add the allocated sensor to the respective structure
      allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
    }
    else
    {
      sensor_ptr->adjustReliability(0.0);
      res.rmp = load_process_msg.response.rmp;
      res.rmp.errorStack = error_handler_.forwardAndReturn(load_process_msg.response.rmp.errorStack, prefix);
    }

    YAML::Node config;
    config["Sensors"].push_back(*sensor_ptr);

    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
    return;
  }

  // try remote sensors
  for (auto& rs : remote_sensors_)
  {
    TEMOTO_INFO("Looking from: \n%s", rs->toString().c_str());
  }

  sensor_ptr = findSensor(req, remote_sensors_);
  if (sensor_ptr)
  {
    // remote sensor candidate was found, forward the request to the remote sensor manager
    temoto_2::LoadSensor load_sensor_msg;
    load_sensor_msg.request.sensor_type = sensor_ptr->getType();
    load_sensor_msg.request.package_name = sensor_ptr->getPackageName();
    load_sensor_msg.request.executable = sensor_ptr->getExecutable();
    load_sensor_msg.request.output_topics = req.output_topics;

    TEMOTO_INFO("SensorManager is forwarding request: '%s', '%s', '%s', reliability %.3f",
                sensor_ptr->getType().c_str(), sensor_ptr->getPackageName().c_str(),
                sensor_ptr->getExecutable().c_str(), sensor_ptr->getReliability());

    if (!resource_manager_.call<temoto_2::LoadSensor>(
            sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, load_sensor_msg,
            sensor_ptr->getTemotoNamespace()))
    {
      // Respond with an error message
      res.rmp.code = 1;
      res.rmp.message = "Failed to call the remote Sensor Manager of "
                      + std::string(sensor_ptr->getTemotoNamespace());
      res.rmp.errorStack = error_handler_.createAndReturn(ErrorCode::SERVICE_REQ_FAIL
                                                        , prefix
                                                        , res.rmp.message);
      return;
    }

    // Check if the request was successful
    if (load_sensor_msg.response.rmp.code == 0)
    {
      TEMOTO_DEBUG("%s Call to remote Sensor Manager was sucessful.", prefix.c_str());
      res = load_sensor_msg.response;
      allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
    }
    else
    {
      res.rmp = load_sensor_msg.response.rmp;
      res.rmp.errorStack = error_handler_.forwardAndReturn(res.rmp.errorStack, prefix);
    }
    return;
  }

  // No suitable local nor remote sensor was found
  res.rmp.code = 1;
  res.rmp.message = "SensorManager did not find a suitable sensor.";
  res.rmp.errorStack = error_handler_.createAndReturn(ErrorCode::RESOURCE_LOAD_FAIL
                                                    , prefix
                                                    , res.rmp.message);
}

// TODO: rename "stopSensorCb" to "unloadSensorCb"
void SensorManager::stopSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  TEMOTO_DEBUG("%s received a request to stop sensor with id '%ld'", prefix.c_str(),
               res.rmp.resource_id);
  allocated_sensors_.erase(res.rmp.resource_id);
  return;
}

SensorInfoPtr SensorManager::findSensor(temoto_2::LoadSensor::Request& req
                                      , const SensorInfoPtrs& sensor_infos)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  // Local list of devices that follow the requirements
  std::vector<SensorInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(sensor_infos.begin()
                       , sensor_infos.end()
                       , std::back_inserter(candidates)
                       , [&](const SensorInfoPtr& s)
                         {
                           return s->getType() == req.sensor_type;
                         });
  
  // The requested type of sensor is not available
  if (candidates.empty())
  {
    return NULL;
  }
 
  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(),
                            [&](SensorInfoPtr s)
                            {
                              return s->getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfoPtr s)
                            {
                              return s->getExecutable() != req.executable;
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfoPtr s)
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
  std::sort(candidates.begin(), it_end, [](SensorInfoPtr& s1, SensorInfoPtr& s2) {
    return s1->getReliability() > s2->getReliability();
  });

  if (candidates.begin() == it_end)
  {
    // Sensor with the requested criteria was not found.
    return NULL;
  }

  // Return the first sensor of the requested type.
  return candidates.front();
}


SensorInfoPtrs SensorManager::parseSensors(const YAML::Node& config)
{
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
  std::vector<SensorInfoPtr> sensors;

//  TEMOTO_DEBUG("%s CONFIG NODE:%d %s", prefix.c_str(), config.Type(), Dump(config).c_str());
  if (!config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("%s Unable to parse 'Sensors' key from config.", prefix.c_str());
    return sensors;
  }

  YAML::Node sensors_node = config["Sensors"];
 // TEMOTO_DEBUG("%s SENSORS NODE:%d", prefix.c_str(), sensors_node.Type());
  if (!sensors_node.IsSequence())
  {
    TEMOTO_WARN("%s The given config does not contain sequence of sensors.", prefix.c_str());
    // TODO Throw
    return sensors;
  }

  TEMOTO_DEBUG("%s Parsing %lu sensors.", prefix.c_str(), sensors_node.size());

  // go over each sensor node in the sequence
  for (YAML::const_iterator node_it = sensors_node.begin(); node_it != sensors_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("%s Unable to parse the sensor. Parameters in YAML have to be specified in "
                   "key-value pairs.",
                   prefix.c_str());
      continue;
    }

    try
    {
      SensorInfo sensor = node_it->as<SensorInfo>();
      if (std::count_if(sensors.begin(), sensors.end(),
                        [&](const SensorInfoPtr& s) { return *s == sensor; }) == 0)
      {
        // OK, this is unique pointer, add it to the sensors vector.
        sensors.emplace_back(std::make_shared<SensorInfo>(sensor));
      }
      else
      {
        TEMOTO_WARN("%s Ignoring duplicate of sensor '%s'.", prefix.c_str(),
                    sensor.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<SensorInfo> e)
    {
      TEMOTO_WARN("%s Failed to parse SensorInfo from config.", prefix.c_str());
      continue;
    }
  }
  return sensors;
}

}  // sensor_manager namespace
