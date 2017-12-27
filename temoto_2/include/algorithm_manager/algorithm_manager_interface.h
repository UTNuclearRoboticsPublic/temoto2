#ifndef SENSOR_MANAGER_INTERFACE_H
#define SENSOR_MANAGER_INTERFACE_H

#include "common/base_subsystem.h"
#include "TTP/base_task/base_task.h"
#include "sensor_manager/sensor_manager_services.h"
#include "rmp/resource_manager.h"
#include <memory> //unique_ptr


template <class OwnerTask>
class SensorManagerInterface : BaseSubsystem
{
public:
  /**
   * @brief SensorManagerInterface
   */
  SensorManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(TTP::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();

    name_ = task->getName() + "/sensor_manager_interface";

    resource_manager_ = std::unique_ptr<rmp::ResourceManager<SensorManagerInterface>>(new rmp::ResourceManager<SensorManagerInterface>(name_, this));
    resource_manager_->registerStatusCb(&SensorManagerInterface::statusInfoCb);
  }


  /**
   * @brief startSensor
   * @param sensor_type
   */
  std::string startSensor(std::string sensor_type)
  {
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    try
    {
      return startSensor(sensor_type, "", "");
    }
    catch (error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief startSensor
   * @param sensor_type
   * @param package_name
   * @param ros_program_name
   */
  std::string startSensor(std::string sensor_type, std::string package_name,
                          std::string ros_program_name)
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    // Fill out the "StartSensorRequest" request
    temoto_2::LoadSensor srv_msg;
    srv_msg.request.sensor_type = sensor_type;
    srv_msg.request.package_name = package_name;
    srv_msg.request.executable = ros_program_name;

    // Call the server
    if (!resource_manager_->template call<temoto_2::LoadSensor>(
            sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, srv_msg))
    {
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call service");
    }

    // If the request was fulfilled, then add the srv to the list of allocated sensors
    if (srv_msg.response.rmp.code == 0)
    {
      allocated_sensors_.push_back(srv_msg);
      return srv_msg.response.topic;
    }
    else
    {
      //\TODO: Forward
      throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Unsuccessful call to sensor manager");
    }
  }

  /**
   * @brief stopSensor
   * @param sensor_type
   * @param package_name
   * @param ros_program_name
   */
  void stopSensor(std::string sensor_type, std::string package_name, std::string ros_program_name)
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    // Find all instances where request part matches of what was given and unload each resource
    temoto_2::LoadSensor::Request req;
    req.sensor_type = sensor_type;
    req.package_name = package_name;
    req.executable = ros_program_name;

    auto cur_sensor_it = allocated_sensors_.begin();
    while (cur_sensor_it != allocated_sensors_.end())
    {
      // The == operator used in the lambda function is defined in
      // sensor manager services header
      auto found_sensor_it = std::find_if(
          cur_sensor_it, allocated_sensors_.end(),
          [&](const temoto_2::LoadSensor& srv_msg) -> bool { return srv_msg.request == req; });
      if (found_sensor_it != allocated_sensors_.end())
      {
        // do the unloading
        resource_manager_->unloadClientResource(found_sensor_it->response.rmp.resource_id);
        cur_sensor_it = found_sensor_it;
      }
      else if (cur_sensor_it == allocated_sensors_.begin())
      {
        throw CREATE_ERROR(error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not loaded.");
      }
    }
  }

  void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
    validateInterface(prefix);

    TEMOTO_DEBUG("%s status info was received", prefix.c_str());
    TEMOTO_DEBUG_STREAM(srv.request);
    // if any resource should fail, just unload it and try again
    // there is a chance that sensor manager gives us better sensor this time
    if (srv.request.status_code == rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("Sensor manager interface detected a sensor failure. Unloading and "
                                "trying again");
      auto sens_it = std::find_if(allocated_sensors_.begin(), allocated_sensors_.end(),
                                  [&](const temoto_2::LoadSensor& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });
      if (sens_it != allocated_sensors_.end())
      {
        TEMOTO_DEBUG("Unloading");
        resource_manager_->unloadClientResource(sens_it->response.rmp.resource_id);
        TEMOTO_DEBUG("Asking the same sensor again");

        // this call automatically updates the response in allocated sensors vec
        if (!resource_manager_->template call<temoto_2::LoadSensor>(
                sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, *sens_it))
        {
          throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Failed to call service");
        }

        if (sens_it->response.rmp.code == 0)
        {
          // @TODO: send somehow topic to whoever is using this thing
          // or do topic remapping
        }
        else
        {
          throw CREATE_ERROR(error::Code::SERVICE_REQ_FAIL, "Unsuccessful call to sensor manager: ");
        }
      }
      else
      {
      }
    }
  }

  ~SensorManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

private:
  std::string name_;
  std::vector<temoto_2::LoadSensor> allocated_sensors_;
  std::unique_ptr<rmp::ResourceManager<SensorManagerInterface>> resource_manager_;

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::NOT_INITIALIZED, "Interface is not initalized.");
    }
  }
};

#endif