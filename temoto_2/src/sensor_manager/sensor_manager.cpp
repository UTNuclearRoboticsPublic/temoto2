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
//#include "sensor_manager/sensor_manager_services.h"
//#include "process_manager/process_manager_services.h"
//#include <sstream>

namespace sensor_manager
{
SensorManager::SensorManager() : resource_manager_(srv_name::MANAGER, this)
{
  // Start the server
  resource_manager_.addServer<temoto_2::LoadSensor>(srv_name::SERVER, &SensorManager::startSensorCb,
                                                    &SensorManager::stopSensorCb);
  resource_manager_.registerStatusCb(&SensorManager::statusCb);

  list_devices_server_ = nh_.advertiseService(srv_name::MANAGER + "/list_devices",
                                              &SensorManager::listDevicesCb, this);
  ROS_INFO("[SensorManager::SensorManager::SensorManager] all services initialized, manager is "
           "good to go");
}

SensorManager::~SensorManager()
{
  ROS_INFO("Destroying sensor_manager.");
}


void SensorManager::statusCb(temoto_2::RMPStatus& status_msg)
{
        ROS_WARN("[SensorManager::statusCb] ...");
    // adjust package reliability when someone reported that it has failed.
    if (status_msg.status_code == rmp::status_codes::FAILED)
    {
      auto it = allocated_sensors_.find(status_msg.resource_id);
      if(it != allocated_sensors_.end())
      {
        ROS_WARN("[SensorManager::statusCb] Sensor failed, adjusting package reliability ...");
        it->second->adjustReliability(0.0);
      }
      
    }
}


bool SensorManager::listDevicesCb(temoto_2::ListDevices::Request& req,
                                  temoto_2::ListDevices::Response& res)
{
  // std::vector <std::string> devList;

  // Find the devices with the required type
  for (auto& entry : pkg_infos_)
  {
    if (entry->getType().compare(req.type) == 0)
    {
      res.list.push_back(entry->getName());
    }
  }

  return true;
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
  ROS_INFO("[SensorManager::start_sensor_cb] received a request to start a '%s': '%s', '%s'",
           req.sensor_type.c_str(), req.package_name.c_str(), req.executable.c_str());

  // Create an empty message that will be filled out by "findSensor" function
  temoto_2::LoadProcess load_process_msg;

  // Find the suitable sensor
  auto pkg_ptr = findSensor(load_process_msg.request, res, req.sensor_type, req.package_name, req.executable);
  if (pkg_ptr)
  {
    ROS_INFO("[SensorManager::start_sensor_cb] Found a suitable sensor. Calling to LoadProcess "
             "server: '%s', '%s', '%s'",
             load_process_msg.request.action.c_str(), load_process_msg.request.package_name.c_str(),
             load_process_msg.request.executable.c_str());
    if (resource_manager_.call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER,
                                                      process_manager::srv_name::SERVER,
                                                      load_process_msg))
    {
      ROS_INFO("[SensorManager::start_sensor_cb] Call to server on ProcessManager was sucessful.");
    }
    else
    {
      ROS_ERROR("[SensorManager::start_sensor_cb] Failed to call server on ProcessManager ...");
      return;
    }
    res.rmp.code = load_process_msg.response.rmp.code;
    res.rmp.message = load_process_msg.response.rmp.message;

    // Increase or decrease reliability depending on the return code
    (res.rmp.code == 0) ? pkg_ptr->adjustReliability(1.0) : pkg_ptr->adjustReliability(0.0);

    ROS_INFO("[SensorManager::start_sensor_cb] LoadProcess server responded: '%s'",
             res.rmp.message.c_str());
  }
  else
  {
    res.package_name = req.package_name;
    res.executable = "";
    res.topic = "";
    res.rmp.code = 1;
    res.rmp.message = "Suitable sensor was not found. Aborting the request";
    ROS_ERROR("[SensorManager::start_sensor_cb] %s", res.rmp.message.c_str());
  }
}

void SensorManager::stopSensorCb(temoto_2::LoadSensor::Request& req,
                                 temoto_2::LoadSensor::Response& res)
{
  ROS_INFO("[SensorManager::stop_sensor_cb] received a request to stop sensor with id '%ld'",
           res.rmp.resource_id);
  return;
}

PackageInfoPtr SensorManager::findSensor(temoto_2::LoadProcess::Request& ret,
                               temoto_2::LoadSensor::Response& retstartSensor, std::string type,
                               std::string name, std::string executable)
{
  // Local list of devices that follow the requirements
  std::vector<PackageInfoPtr> candidates;
  std::vector<PackageInfoPtr> candidatesLocal;

  // Find the devices that follow the "type" criteria
  for (auto pkg_ptr : pkg_infos_)
  {
    if (pkg_ptr->getType().compare(type) == 0)
    {
      candidates.push_back(pkg_ptr);
    }
  }

  // If the list is empty, leave the req empty
  if (candidates.empty())
  {
    return NULL;
  }

  std::sort(candidates.begin(), candidates.end(), [](PackageInfoPtr& pkg_ptr1, PackageInfoPtr& pkg_ptr2)
      {
return pkg_ptr1->getReliability() > pkg_ptr2->getReliability();
      });


  // Check if a name was specified
  if (name != "")
  {
    // Filter out the devices that follow the "name" criteria
    for (auto& entry : candidates)
    {
      if (entry->getName() == name)
      {
        if(entry->getLaunchables().size()!=0)
          ROS_INFO("SENSOR: %s", entry->getLaunchables().begin()->first.c_str());
        candidatesLocal.push_back(entry);
      }
    }

    // Copy the contents of "candidatesLocal" into "candidates"
    candidates = candidatesLocal;

    // If the list is empty, return an empty response
    if (candidates.empty())
    {
      return NULL;
    }
  }

  else
  {
    // Get the name of the package
    ret.package_name = candidates[0]->getName();

    // Check for runnables
    if (!candidates[0]->getRunnables().empty())
    {
      ret.action = "rosrun";
      ret.executable = candidates[0]->getRunnables().begin()->first;
      retstartSensor.topic = candidates[0]->getRunnables().begin()->second;
    }

    else if (!candidates[0]->getLaunchables().empty())
    {
      ret.action = "roslaunch";
      ret.executable = candidates[0]->getLaunchables().begin()->first;
      retstartSensor.topic = candidates[0]->getLaunchables().begin()->second;
    }

    // The name of the topic that this particular runnable publishes to
    retstartSensor.package_name = ret.package_name;
    retstartSensor.executable = ret.executable;

    return candidates[0];
  }

  // Check if the runnable/launchable/executable was specified
  if (executable != "")
  {
    // Clear out the "candidatesLocal" list
    candidatesLocal.clear();

    // Filter out the devices that follow the "executable" criteria
    for (auto& entry : candidates)
    {
      // Get the local runnables
      std::map<std::string, std::string> runnables_entry = entry->getRunnables();

      // Check if the required runnable exists in the list of runnables
      if (runnables_entry.find(executable) != runnables_entry.end())
      {
        candidatesLocal.push_back(entry);
      }

      // Get the local launchables
      std::map<std::string, std::string> launchables_entry = entry->getLaunchables();

      // Check if the required runnable exists in the list of runnables
      if (launchables_entry.find(executable) != launchables_entry.end())
      {
        candidatesLocal.push_back(entry);
      }
    }

    // Copy the contents of "candidatesLocal" into "candidates"
    candidates = candidatesLocal;

    // If the list is empty, return an empty response
    if (candidates.empty())
    {
      return NULL;
    }
  }

  else
  {
    // Return the first "runnable" of the element in the "candidates" list
    ret.action = "roslaunch";
    ret.package_name = name;
    ret.executable = candidates[0]->getRunnables().begin()->first;

    // The name of the topic that this particular runnable publishes to
    retstartSensor.package_name = name;
    retstartSensor.executable = ret.executable;
    retstartSensor.topic = candidates[0]->getRunnables().begin()->second;

    return candidates[0];
  }

  // If all above constraints were satisfied, then:
  ret.action = "roslaunch";
  ret.package_name = name;
  ret.executable = executable;

  // The name of the topic that this particular runnable publishes to
  retstartSensor.package_name = name;
  retstartSensor.executable = executable;

  // Check if runnables were found
  if (!candidates[0]->getRunnables().empty())
  {
    retstartSensor.topic = candidates[0]->getRunnables()[executable];
  }
  else if (!candidates[0]->getLaunchables().empty())
  {
    retstartSensor.topic = candidates[0]->getLaunchables()[executable];
  }
  return candidates[0];
}

}  // sensor_manager namespace
