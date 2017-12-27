#ifndef SENSOR_MANAGER_INTERFACE_H
#define SENSOR_MANAGER_INTERFACE_H

#include "common/base_subsystem.h"
#include "common/topic_container.h"

#include "TTP/base_task/base_task.h"
#include "sensor_manager/sensor_manager_services.h"
#include "rmp/resource_manager.h"
#include <memory> //unique_ptr

/**
 * @brief The AlgorithmTopicsReq class
 */
class SensorTopicsReq : public TopicContainer
{
  /* DELIBERATELY EMPTY */
};

/**
 * @brief The AlgorithmTopicsRes class
 */
class SensorTopicsRes : public TopicContainer
{
  /* DELIBERATELY EMPTY */
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                          SENSOR MANAGER INTERFACE
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace sensor_manager
{

template <class OwnerTask>
class SensorManagerInterface : public BaseSubsystem
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
   * @return
   */
  SensorTopicsRes startSensor(const std::string& sensor_type)
  {

    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return startSensor(sensor_type, "", "", SensorTopicsReq());
  }

  /**
   * @brief startSensor
   * @param sensor_type
   * @param topics
   * @return
   */
  SensorTopicsRes startSensor(const std::string& sensor_type, const SensorTopicsReq& topics)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return startSensor(sensor_type, "", "", topics);
  }

  /**
   * @brief startSensor
   * @param algorithm_type
   * @param package_name
   * @param ros_program_name
   * @param topics
   * @return
   */
  SensorTopicsRes startSensor(const std::string& sensor_type
                            , const std::string& package_name
                            , const std::string& ros_program_name
                            , const SensorTopicsReq& topics)
  {
    validateInterface();

    // Fill out the "StartSensorRequest" request
    temoto_2::LoadSensor srv_msg;
    srv_msg.request.sensor_type = sensor_type;
    srv_msg.request.package_name = package_name;
    srv_msg.request.executable = ros_program_name;
    srv_msg.request.output_topics = topics.outputTopicsAsKeyValues();

    // Call the server    
    try
    {
      resource_manager_->template call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
                                                             sensor_manager::srv_name::SERVER,
                                                             srv_msg);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    allocated_sensors_.push_back(srv_msg);
    SensorTopicsRes responded_topics;
    responded_topics.setOutputTopicsByKeyValue( srv_msg.response.output_topics );

    return responded_topics;
  }

  /**
   * @brief stopSensor
   * @param sensor_type
   * @param package_name
   * @param ros_program_name
   */
  void stopSensor(std::string sensor_type, std::string package_name, std::string ros_program_name)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Find all instances where request part matches of what was given and unload each resource
    temoto_2::LoadSensor::Request req;
    req.sensor_type = sensor_type;
    req.package_name = package_name;
    req.executable = ros_program_name;

    // The == operator used in the lambda function is defined in
    // sensor manager services header
    auto found_sensor_it = std::find_if(
        allocated_sensors_.begin(),
        allocated_sensors_.end(),
        [&](const temoto_2::LoadSensor& srv_msg) -> bool{ return srv_msg.request == req; });

    if (found_sensor_it == allocated_sensors_.end())
    {
      throw CREATE_ERROR(error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not "
                                                            "loaded.");
    }

    try
    {
      // do the unloading
      resource_manager_->unloadClientResource(found_sensor_it->response.rmp.resource_id);
      allocated_sensors_.erase(found_sensor_it);
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief statusInfoCb
   * @param srv
   */
  void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("status info was received");
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
        try
        {
          resource_manager_->template call<temoto_2::LoadSensor>(
              sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, *sens_it);
        }
        catch(error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }
      else
      {
        throw CREATE_ERROR(error::Code::RESOURCE_NOT_FOUND, "Resource status arrived for a "
                                                            "resource that does not exist.");
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
   * @brief validateInterface
   */
  void validateInterface()
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace

#endif
