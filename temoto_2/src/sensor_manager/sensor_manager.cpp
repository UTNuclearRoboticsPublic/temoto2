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
#include "sensor_manager/sensor_manager.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace sensor_manager
{
SensorManager::SensorManager()
  : BaseSubsystem("sensor_manager", error::Subsystem::SENSOR_MANAGER, __func__)
  ,  resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &SensorManager::syncCb, this)
{
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
      TEMOTO_DEBUG("Added sensor: '%s'.", s->getName().c_str());
    }
    // notify other managers about our sensors
    advertiseLocalSensors();
  }
  else
  {
    TEMOTO_WARN("Failed to read '%s'. Verify that the file exists and the sequence of sensors "
                "is listed under 'Sensors' node.", yaml_filename.c_str());
  }


  TEMOTO_INFO("Sensor manager is ready.");
}

SensorManager::~SensorManager()
{
}

void SensorManager::statusCb(temoto_2::ResourceStatus& srv)
{

  TEMOTO_DEBUG("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_sensors_.find(srv.request.resource_id);
    if (it != allocated_sensors_.end())
    {
      if(it->second->isLocal())
      {
        TEMOTO_WARN("Local sensor failure detected, adjusting reliability.");
        it->second->adjustReliability(0.0);
        YAML::Node config;
        config["Sensors"].push_back(*it->second);

        PayloadType payload;
        payload.data = Dump(config);
        config_syncer_.advertise(payload);
      }
      else
      {
        TEMOTO_WARN("Remote sensor failure detected, doing nothing (sensor will be updated via synchronizer).");
      }
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

    //for (auto& s : remote_sensors_)
    //{
    //  TEMOTO_DEBUG("---------REMOTE SENSOR: \n %s", s->toString().c_str());
    //}

    for (auto& sensor : sensors)
    {
      //TEMOTO_DEBUG("------ COMPARING Sensor \n %s", sensor->toString().c_str());
      // Check if sensor has to be added or updated
      auto it = std::find_if(remote_sensors_.begin(), remote_sensors_.end(),
          [&](const SensorInfoPtr& rs) { return *rs == *sensor; });
      if (it != remote_sensors_.end())
      {
        TEMOTO_DEBUG("Updating remote sensor '%s' at '%s'.", sensor->getName().c_str(),
                     sensor->getTemotoNamespace().c_str());
        *it = sensor; // overwrite found sensor
      }
      else
      {
        TEMOTO_DEBUG("Adding remote sensor '%s' at '%s'.", sensor->getName().c_str(),
                     sensor->getTemotoNamespace().c_str());
        remote_sensors_.push_back(sensor);
      }
    }
  }
}

void SensorManager::advertiseSensor(SensorInfoPtr sensor_ptr)
{
  //TEMOTO_DEBUG("------ Advertising Sensor \n %s", sensor_ptr->toString().c_str());
    YAML::Node config;
    config["Sensors"].push_back(*sensor_ptr);
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
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
  TEMOTO_INFO_STREAM("- - - - - - - - - - - - -\n"
                     << "Received a request to load a sensor: \n" << req << std::endl);

  // Try to find suitable candidate from local sensors
  auto sensor_ptr = findSensor(req, local_sensors_);
  if (sensor_ptr)
  {
    // Try to run the sensor via local Resource Manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = sensor_ptr->getPackageName();
    load_process_msg.request.executable = sensor_ptr->getExecutable();

    // Check if any particular topic types were requested
    if (!req.output_topics.empty())
    {
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
    }
    else
    {
      TopicContainer output_topics;
      output_topics.setOutputTopics(sensor_ptr->getOutputTopics());

      // Translate all topics to absolute
      res.output_topics.clear();
      for (const auto& output_topic : sensor_ptr->getOutputTopics())
      {
        diagnostic_msgs::KeyValue topic_msg;
        topic_msg.key = output_topic.first;
        topic_msg.value = common::getAbsolutePath(output_topic.second);
        res.output_topics.push_back(topic_msg);
      }
    }
    TEMOTO_INFO("SensorManager found a suitable local sensor: '%s', '%s', '%s', reliability %.3f",
                load_process_msg.request.action.c_str(),
                load_process_msg.request.package_name.c_str(),
                load_process_msg.request.executable.c_str(), sensor_ptr->getReliability());

    try
    {
      resource_manager_.call<temoto_2::LoadProcess>(
          process_manager::srv_name::MANAGER, process_manager::srv_name::SERVER, load_process_msg, rmp::FailureBehavior::NONE);
      TEMOTO_DEBUG("Call to ProcessManager was sucessful.");

      // Fill out the response about which particular sensor was chosen
      res.package_name = sensor_ptr->getPackageName();
      res.executable = sensor_ptr->getExecutable();
      res.rmp = load_process_msg.response.rmp;

      sensor_ptr->adjustReliability(1.0);
      advertiseSensor(sensor_ptr);
    }
    catch(error::ErrorStack& error_stack)
    { 
      if (error_stack.front().code != static_cast<int>(error::Code::SERVICE_REQ_FAIL))
      {
        sensor_ptr->adjustReliability(0.0);
        advertiseSensor(sensor_ptr);
      }
      throw FORWARD_ERROR(error_stack);
    }

    allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);

    return;
  }

  // try remote sensors
//  for (auto& rs : remote_sensors_)
//  {
//    TEMOTO_INFO("Looking from: \n%s", rs->toString().c_str());
//  }

  sensor_ptr = findSensor(req, remote_sensors_);
  if (sensor_ptr)
  {
    // remote sensor candidate was found, forward the request to the remote sensor manager
    temoto_2::LoadSensor load_sensor_msg;
    load_sensor_msg.request.sensor_type = sensor_ptr->getType();
    load_sensor_msg.request.package_name = sensor_ptr->getPackageName();
    load_sensor_msg.request.executable = sensor_ptr->getExecutable();
    load_sensor_msg.request.output_topics = req.output_topics;

    TEMOTO_INFO("Sensor Manager is forwarding request: '%s', '%s', '%s', reliability %.3f",
                sensor_ptr->getType().c_str(), sensor_ptr->getPackageName().c_str(),
                sensor_ptr->getExecutable().c_str(), sensor_ptr->getReliability());

    try
    {
      resource_manager_.call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                   sensor_manager::srv_name::SERVER,
                                                   load_sensor_msg,
                                                   rmp::FailureBehavior::NONE,
                                                   sensor_ptr->getTemotoNamespace());

      TEMOTO_DEBUG("Call to remote SensorManager was sucessful.");
      res = load_sensor_msg.response;
      allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
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
    throw CREATE_ERROR(error::Code::SENSOR_NOT_FOUND, "SensorManager did not find a suitable sensor.");
  }
}

// TODO: rename "stopSensorCb" to "unloadSensorCb"
void SensorManager::stopSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  TEMOTO_DEBUG("received a request to stop sensor with id '%ld'", res.rmp.resource_id);
  allocated_sensors_.erase(res.rmp.resource_id);
  return;
}

SensorInfoPtr SensorManager::findSensor(temoto_2::LoadSensor::Request& req
                                      , const SensorInfoPtrs& sensor_infos)
{
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
                              if (s->getOutputTopics().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s->getOutputTopics();

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
  std::vector<SensorInfoPtr> sensors;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Sensors' key from config.");
    return sensors;
  }

  YAML::Node sensors_node = config["Sensors"];
  if (!sensors_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of sensors.");
    return sensors;
  }

  TEMOTO_DEBUG("Parsing %lu sensors.", sensors_node.size());

  // go over each sensor node in the sequence
  for (YAML::const_iterator node_it = sensors_node.begin(); node_it != sensors_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the sensor. Parameters in YAML have to be specified in "
                   "key-value pairs.");
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
        //TEMOTO_DEBUG_STREAM("####### PARSED SENSOR: #######\n" << sensors.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of sensor '%s'.", sensor.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<SensorInfo> e)
    {
      TEMOTO_WARN("Failed to parse SensorInfo from config.");
      continue;
    }
  }
  return sensors;
}

}  // sensor_manager namespace
