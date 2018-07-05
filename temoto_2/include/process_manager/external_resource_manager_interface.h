#ifndef EXTERNAL_RESOURCE_MANAGER_INTERFACE_H
#define EXTERNAL_RESOURCE_MANAGER_INTERFACE_H

#include "common/base_subsystem.h"

#include "TTP/base_task/base_task.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include <memory> //unique_ptr


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                   EXTERNAL RESOURCE MANAGER INTERFACE
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace external_resource_manager
{

template <class OwnerTask>
class ExternalResourceManagerInterface : public BaseSubsystem
{
public:
  /**
   * @brief ExternalResourceManagerInterface
   */
  ExternalResourceManagerInterface()
  {
    class_name_ = __func__;
  }

  /**
   * @brief initialize
   * @param task
   */
  void initialize(OwnerTask* task)
  {
    owner_instance_ = task;
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/external_resource_manager_interface";

    resource_manager_ = std::unique_ptr<rmp::ResourceManager<ExternalResourceManagerInterface>>(new rmp::ResourceManager<ExternalResourceManagerInterface>(name_, this));
    resource_manager_->registerStatusCb(&ExternalResourceManagerInterface::statusInfoCb);
  }

  /**
   * @brief loadResource
   * @param package_name
   * @param ros_program_name
   * @param args
   */
  void loadResource(const std::string& package_name,
                    const std::string& ros_program_name,
                    const std::string& args = "")
  {
    validateInterface();

    // Fill out the "LoadProcess" request
    temoto_2::LoadProcess load_resource_msg;
    load_resource_msg.request.action = process_manager::action::ROS_EXECUTE;
    load_resource_msg.request.package_name = package_name;
    load_resource_msg.request.executable = ros_program_name;
    load_resource_msg.request.args = args;

    // Call the server
    try
    {
//      resource_manager_->template call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
//                                                             sensor_manager::srv_name::SERVER,
//                                                             load_resource_msg);

       resource_manager_->template call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER,
                                                               process_manager::srv_name::SERVER,
                                                               load_resource_msg,
                                                               rmp::FailureBehavior::NONE);
    }
    catch(error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    allocated_external_resources_.push_back(load_resource_msg);
  }

//  /**
//   * @brief stopSensor
//   * @param sensor_type
//   * @param package_name
//   * @param ros_program_name
//   */
//  void unloadResource(std::string sensor_type, std::string package_name, std::string ros_program_name)
//  {
//    try
//    {
//      validateInterface();
//    }
//    catch (error::ErrorStack& error_stack)
//    {
//      throw FORWARD_ERROR(error_stack);
//    }

//    // Find all instances where request part matches of what was given and unload each resource
//    temoto_2::LoadSensor::Request req;
//    req.sensor_type = sensor_type;
//    req.package_name = package_name;
//    req.executable = ros_program_name;

//    // The == operator used in the lambda function is defined in
//    // sensor manager services header
//    auto found_sensor_it = std::find_if(
//        allocated_sensors_.begin(),
//        allocated_sensors_.end(),
//        [&](const temoto_2::LoadSensor& load_resource_msg) -> bool{ return load_resource_msg.request == req; });

//    if (found_sensor_it == allocated_sensors_.end())
//    {
//      throw CREATE_ERROR(error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not "
//                                                            "loaded.");
//    }

//    try
//    {
//      // do the unloading
//      resource_manager_->unloadClientResource(found_sensor_it->response.rmp.resource_id);
//      allocated_sensors_.erase(found_sensor_it);
//    }
//    catch (error::ErrorStack& error_stack)
//    {
//      throw FORWARD_ERROR(error_stack);
//    }
//  }

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

    TEMOTO_INFO_STREAM("status info was received");
    TEMOTO_INFO_STREAM(srv.request);

    /*
     * Check if the owner action has a status routine defined
     */
    if (update_callback_)
    {
      (owner_instance_->*update_callback_)(false);
      return;
    }

//    /*
//     * if any resource should fail, just unload it and load it again
//     * there is a chance that sensor manager gives us better sensor this time
//     */
//    if (srv.request.status_code == rmp::status_codes::FAILED)
//    {
//      TEMOTO_WARN("Sensor manager interface detected a sensor failure. Unloading and "
//                                "trying again");
//      auto sens_it = std::find_if(allocated_sensors_.begin(), allocated_sensors_.end(),
//                                  [&](const temoto_2::LoadSensor& sens) -> bool {
//                                    return sens.response.rmp.resource_id == srv.request.resource_id;
//                                  });
//      if (sens_it != allocated_sensors_.end())
//      {
//        TEMOTO_DEBUG("Unloading");
//        resource_manager_->unloadClientResource(sens_it->response.rmp.resource_id);
//        TEMOTO_DEBUG("Asking the same sensor again");

//        // this call automatically updates the response in allocated sensors vec
//        try
//        {
//          resource_manager_->template call<temoto_2::LoadSensor>(sensor_manager::srv_name::MANAGER,
//                                                                 sensor_manager::srv_name::SERVER,
//                                                                 *sens_it);
//        }
//        catch(error::ErrorStack& error_stack)
//        {
//          SEND_ERROR(error_stack);
//        }
//      }
//      else
//      {
//        throw CREATE_ERROR(error::Code::RESOURCE_NOT_FOUND, "Resource status arrived for a "
//                                                            "resource that does not exist.");
//      }
//    }
  }

  /**
   * @brief registerUpdateCallback
   */
  void registerUpdateCallback( void (OwnerTask::*callback )(bool))
  {
    update_callback_ = callback;
  }

  ~ExternalResourceManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

private:
  std::string name_;
  std::vector<temoto_2::LoadProcess> allocated_external_resources_;
  std::unique_ptr<rmp::ResourceManager<ExternalResourceManagerInterface>> resource_manager_;

  void(OwnerTask::*update_callback_)(bool) = NULL;
  OwnerTask* owner_instance_;

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
