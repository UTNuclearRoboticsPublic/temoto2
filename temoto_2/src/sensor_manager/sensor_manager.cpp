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

//#include "PackageInfo/PackageInfo.h"

#include "core/common.h"
#include "sensor_manager/sensor_manager.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
//#include "sensor_manager/sensor_manager_services.h"
//#include "process_manager/process_manager_services.h"
//#include <sstream>

namespace sensor_manager
{
SensorManager::SensorManager()
  : resource_manager_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &SensorManager::syncCb, this)
{
  log_class_ = "";
  log_subsys_ = "sensor_manager";
  log_group_ = "sensor_manager";
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");

  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>(srv_name::SERVER, &SensorManager::startSensorCb,
                                                    &SensorManager::stopSensorCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&SensorManager::statusCb);

  config_syncer_.requestRemoteConfigs();

  //  list_devices_server_ = nh_.advertiseService(srv_name::MANAGER + "/list_devices",
  //                                              &SensorManager::listDevicesCb, this);

  // Read the sensors for this manager. 
  /// \todo HARDCODED PATH
  std::string yaml_filename = "/home/veix/catkin_ws/src/temoto2/temoto_2/conf/" +
                              common::getTemotoNamespace() + "_sensors.yaml";
  std::ifstream in(yaml_filename);
  YAML::Node config = YAML::Load(in)["Sensors"];
  if (config.IsSequence())
  {
    local_sensors_ = parseSensors(config);
  }
  else
  {
    TEMOTO_WARN("%s Failed to read '%s'. Verify that the file exists and the sequence of sensors "
                "is listed under 'Sensors' node.",
                prefix.c_str(), yaml_filename.c_str());
  }

  // publish sync message which causes each sensor manager to publish its sensors.
 // SensorInfo dummy_sensor;
 // sync_pub_.publish(dummy_sensor.getSyncMsg(sync_action::GET_SENSORS));

  TEMOTO_INFO("Sensor manager is ready.");
}

SensorManager::~SensorManager()
{
}

void SensorManager::statusCb(temoto_2::ResourceStatus& srv)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s Status received.", prefix.c_str());
  // adjust package reliability when someone reported that it has failed.
  if (srv.request.status_code == rmp::status_codes::FAILED)
  {
    auto it = allocated_sensors_.find(srv.request.resource_id);
    if (it != allocated_sensors_.end())
    {
      TEMOTO_WARN("Sensor failure detected, adjusting reliability.");
      it->second->adjustReliability(0.0);
      // Send out the information about the new sensor.
      //sync_pub_.publish(it->second->getSyncMsg(sync_action::UPDATE));
      //config_syncer_.sendUpdate(it->second->getSyncMsg().sensor_name);
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

void SensorManager::syncCb(const temoto_2::ConfigSync& msg)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    // publish all local sensors
    YAML::Node config;
    for(auto& s : local_sensors_) 
    {
      if(s->getTemotoNamespace() == common::getTemotoNamespace())
      {
        config["Sensors"].push_back(*s);
      }
    }
    config_syncer_.sendUpdate(config);
    return;
  }

  if (msg.action == rmp::sync_action::UPDATE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(msg.config);
    SensorInfo sensors = parseSensors(config);

    // Check if sensor has to be added or updated
    auto it = std::find_if(sensors_.begin(), sensors_.end(),
                           [&](const SensorInfoPtr& s) { return *s == sensor; });
    if (it != sensors_.end())
    {
      TEMOTO_DEBUG("%s Updating remote sensor '%s' at '%s'.", prefix.c_str(),
                   sensor.getName().c_str(), sensor.getTemotoNamespace().c_str());
      *it = std::make_shared<SensorInfo>(sensor); // overwrite found sensor
    }
    else
    {
      TEMOTO_DEBUG("%s Adding remote sensor '%s' at '%s'.", prefix.c_str(),
                   sensor.getName().c_str(), sensor.getTemotoNamespace().c_str());
      sensors_.emplace_back(std::make_shared<SensorInfo>(sensor));
    }
  }
}

void SensorManager::addSensor(const SensorInfo& sensor)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  auto it = std::find_if(local_sensors_.begin(), local_sensors_.end(),
                         [&](const SensorInfoPtr& s) { return *s == sensor; });
  if (it != local_sensors_.end())
  {
    TEMOTO_WARN("%s The sensor '%s' is already added.", prefix.c_str(), sensor.getName().c_str());
    return;
  }

  TEMOTO_DEBUG("%s Adding sensor: \n%s", prefix.c_str(), sensor.toString().c_str());

  // Add pointer to sensors list.
  local_sensors_.emplace_back(std::make_shared<SensorInfo>(sensor));

  // Send out information about the new sensor.
 // sync_pub_.publish(local_sensors_.back()->getSyncMsg(sync_action::UPDATE));
 
}

/*
 * Callback to a service that executes/runs a requested device
 * and sends back the topic that the device is publishing to
 * THIS IS LIKELY A GENERIC FUNCTION THAT WILL BE USED ALSO BY
 * OTHER MANAGERS
 */

/**
 * @brief Start node service
 * @param req
 * @param res
 * @return
 */
void SensorManager::startSensorCb(temoto_2::LoadSensor::Request& req,
                                  temoto_2::LoadSensor::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s received a request to start '%s': '%s', '%s'", prefix.c_str(),
               req.sensor_type.c_str(), req.package_name.c_str(), req.executable.c_str());

  // Try to find suitable candidate from local sensors
  auto sensor_ptr = findSensor(req.sensor_type, req.package_name, req.executable, local_sensors_);
  if (sensor_ptr)
  {
    // local sensor found, make a call to the local resource manager
    temoto_2::LoadProcess load_process_msg;
    load_process_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = sensor_ptr->getPackageName();
    load_process_msg.request.executable = sensor_ptr->getExecutable();

    TEMOTO_INFO("SensorManager found a suitable local sensor: '%s', '%s', '%s', reliability %.3f",
                load_process_msg.request.action.c_str(),
                load_process_msg.request.package_name.c_str(),
                load_process_msg.request.executable.c_str(), sensor_ptr->getReliability());

    if (resource_manager_.call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER,
                                                      process_manager::srv_name::SERVER,
                                                      load_process_msg))
    {
      TEMOTO_DEBUG("%s Call to ProcessManager was sucessful.", prefix.c_str());
      // fill in the response
      res.package_name = sensor_ptr->getPackageName();
      res.topic = sensor_ptr->getTopic();
      res.executable = sensor_ptr->getExecutable();
      res.rmp.code = load_process_msg.response.rmp.code;
      res.rmp.message = load_process_msg.response.rmp.message;
    }
    else
    {
      TEMOTO_ERROR("%s Failed to call the ProcessManager.", prefix.c_str());
      return;
    }

    // Increase or decrease the reliability depending on the return code
    // and send the update to other managers.
    (res.rmp.code == 0) ? sensor_ptr->adjustReliability(1.0) : sensor_ptr->adjustReliability(0.0);
    allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
    // sync_pub_.publish(sensor_ptr->getSyncMsg(sync_action::UPDATE));
    return;
  }

  // try remote sensors
  sensor_ptr = findSensor(req.sensor_type, req.package_name, req.executable, remote_sensors_);
  if (sensor_ptr)
  {
    // remote sensor candidate was found, forward the request to the remote sensor manager
    temoto_2::LoadSensor load_sensor_msg;
    load_sensor_msg.request.sensor_type = sensor_ptr->getType();
    load_sensor_msg.request.package_name = sensor_ptr->getPackageName();
    load_sensor_msg.request.executable = sensor_ptr->getExecutable();
    TEMOTO_INFO("SensorManager is forwarding request: '%s', '%s', '%s', reliability %.3f",
                sensor_ptr->getType().c_str(), sensor_ptr->getPackageName().c_str(),
                sensor_ptr->getExecutable().c_str(), sensor_ptr->getReliability());

    if (resource_manager_.call<temoto_2::LoadSensor>(
            sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, load_sensor_msg,
            sensor_ptr->getTemotoNamespace()))
    {
      TEMOTO_DEBUG("%s Call to remote SensorManager was sucessful.", prefix.c_str());
      res.rmp.code = load_sensor_msg.response.rmp.code;
      res.rmp.message = load_sensor_msg.response.rmp.message;
      allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
    }
    else
    {
      TEMOTO_ERROR("%s Failed to call the remote SensorManager.", prefix.c_str());
      return;
    }
    return;
  }

  // no suitable local nor remote sensor was found
  res.package_name = req.package_name;
  res.executable = "";
  res.topic = "";
  res.rmp.code = 1;
  res.rmp.message = "SensorManager did not find a suitable sensor.";
  TEMOTO_ERROR("%s %s", prefix.c_str(), res.rmp.message.c_str());
}

void SensorManager::stopSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s received a request to stop sensor with id '%ld'", prefix.c_str(),
               res.rmp.resource_id);
  allocated_sensors_.erase(res.rmp.resource_id);
  return;
}

SensorInfoPtr SensorManager::findSensor(std::string type, std::string package_name,
                                        std::string executable, const std::vector<SensorInfoPtr>& sensors)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  // Local list of devices that follow the requirements
  std::vector<SensorInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(sensors.begin(), sensors.end(), std::back_inserter(candidates),
                         [&](const SensorInfoPtr& s) { return s->getType() == type; });
  
  TEMOTO_INFO("Local candidates: %lu", candidates.size());

  // The requested type of sensor is not available
  if (candidates.empty())
  {
    return NULL;
  }
 
  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(), [&](SensorInfoPtr s) {
      return s->getPackageName() != package_name;
    });
  }

  // If executable is specified, remove all non-matching candidates
  if (executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end, [&](SensorInfoPtr s) {
      return s->getExecutable() != executable;
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

  // Out of options ... return the first remote sensor of the requested type.
  return candidates.front();
}


std::vector<SensorInfoPtr> SensorManager::parseSensors(const YAML::Node& config)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  std::vector<SensorInfoPtr> sensors;
  YAML::Node sensors_node = config["Sensors"];
  if (!sensors_node.IsSequence())
  {
    TEMOTO_WARN("%s The given config does not contain sequence of sensors.", prefix.c_str());
    return false;
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
}

}  // sensor_manager namespace
