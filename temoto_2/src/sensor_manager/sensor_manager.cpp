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
//#include "sensor_manager/sensor_manager_services.h"
//#include "process_manager/process_manager_services.h"
//#include <sstream>

namespace sensor_manager
{
SensorManager::SensorManager() : resource_manager_(srv_name::MANAGER, this)
{
  log_class_ = "";
  log_subsys_ = "sensor_manager";
  log_group_ = "sensor_manager";

  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>(srv_name::SERVER, &SensorManager::startSensorCb,
                                                    &SensorManager::stopSensorCb);
  // Register callback for status info
  resource_manager_.registerStatusCb(&SensorManager::statusCb);

  //  list_devices_server_ = nh_.advertiseService(srv_name::MANAGER + "/list_devices",
  //                                              &SensorManager::listDevicesCb, this);

  sync_pub_ = nh_.advertise<temoto_2::SensorInfoSync>(srv_name::SYNC_TOPIC, 1);
  sync_sub_ = nh_.subscribe(srv_name::SYNC_TOPIC, 1, &SensorManager::syncCb, this);

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
    }
  }
}

bool SensorManager::listDevicesCb(temoto_2::ListDevices::Request& req,
                                  temoto_2::ListDevices::Response& res)
{
  // std::vector <std::string> devList;

  // Find the devices with the required type
  for (auto& entry : sensors_)
  {
    if (entry->getType() == req.type)
    {
      res.list.push_back(entry->getName());
    }
  }

  return true;
}

void SensorManager::syncCb(const temoto_2::SensorInfoSync& msg)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  TEMOTO_DEBUG("%s Got new info about sensor.", prefix.c_str());

  /**
   * \todo some validation of the msg
   */
}

void SensorManager::addSensor(const SensorInfo& sensor_info)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  auto it = std::find_if(sensors_.begin(), sensors_.end(),
                         [&](const SensorInfoPtr& s) { return *s == sensor_info; });
  if (it != sensors_.end())
  {
    TEMOTO_ERROR("%s The sensor '%s' is already added.", prefix.c_str(), sensor_info.getName().c_str());
    return;
  }
 
  sensors_.emplace_back(std::make_shared<SensorInfo>(sensor_info));
  TEMOTO_DEBUG("%s Added sensor '%s'.", prefix.c_str(), sensor_info.getName().c_str());
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

  // Find the suitable sensor
  auto sensor_ptr = findSensor(req.sensor_type, req.package_name, req.executable);
  if (sensor_ptr)
  {
    // sensor found, check if it's local or remote
    if (sensor_ptr->getTemotoNamespace() == ::common::getTemotoNamespace())
    {
      // this is a local sensor, make a call to the local resource manager
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
    }
    else
    {
      // this is a remote sensor, forward the request to the remote sensor manager
      //

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
      }
      else
      {
        TEMOTO_ERROR("%s Failed to call the remote SensorManager.", prefix.c_str());
        return;
      }
    }

    // Increase or decrease the reliability depending on the return code
    (res.rmp.code == 0) ? sensor_ptr->adjustReliability(1.0) : sensor_ptr->adjustReliability(0.0);

    allocated_sensors_.emplace(res.rmp.resource_id, sensor_ptr);
  }
  else
  {
    // sensor was not found
    res.package_name = req.package_name;
    res.executable = "";
    res.topic = "";
    res.rmp.code = 1;
    res.rmp.message = "SensorManager did not find a suitable sensor.";
    TEMOTO_ERROR("%s %s", prefix.c_str(), res.rmp.message.c_str());
  }
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
                                        std::string executable)
{
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  // Local list of devices that follow the requirements
  std::vector<SensorInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(sensors_.begin(), sensors_.end(), std::back_inserter(candidates),
                         [&](const SensorInfoPtr& s) { return s->getType() == type; });
  
  TEMOTO_INFO("Candidates: %lu", candidates.size());

  // The requested type of sensor is not available, leave the response part empty
  if (candidates.empty())
  {
    return NULL;
  }

  // Sort all sensors of the given type.
  std::sort(candidates.begin(), candidates.end(), [](SensorInfoPtr& s1, SensorInfoPtr& s2) {
    return s1->getReliability() > s2->getReliability();
  });

  if (package_name != "")
  {
    // Search for a sensor, which is local and has the requested name.
    auto it_local = std::find_if(candidates.begin(), candidates.end(), [&](SensorInfoPtr s) {
      return s->getPackageName() == package_name && s->getTemotoNamespace() == ::common::getTemotoNamespace();
    });

    if (it_local != candidates.end())
    {
      TEMOTO_DEBUG("%s found local sensor: %s reliability: %f", prefix.c_str(),
                   (*it_local)->getName().c_str(), (*it_local)->getReliability());
      return *it_local;
    }

    // The sensor was not found locally look for remote sensor with the requested name
    auto it_remote = std::find_if(candidates.begin(), candidates.end(),
                                  [&](SensorInfoPtr s) { return s->getPackageName() == package_name; });
    if (it_remote != candidates.end())
    {
      return *it_remote;
    }

    // Sensor with requested name was not found.
    return NULL;
  }

  // If name is not specified, take the first local sensor
  auto it_local = std::find_if(candidates.begin(), candidates.end(), [&](SensorInfoPtr s) {
    return s->getTemotoNamespace() == ::common::getTemotoNamespace();
  });

  if (it_local != candidates.end())
  {
    return *it_local;
  }

  // Out of options ... return the first remote sensor of the requested type.
  return candidates.front();
}

}  // sensor_manager namespace
