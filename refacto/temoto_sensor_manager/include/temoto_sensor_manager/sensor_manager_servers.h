#ifndef TEMOTO_SENSOR_MANAGER__SENSOR_MANAGER_SERVERS_H
#define TEMOTO_SENSOR_MANAGER__SENSOR_MANAGER_SERVERS_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_sensor_manager/sensor_info_registry.h"
#include "temoto_sensor_manager/sensor_manager_services.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include "std_msgs/String.h"

namespace temoto_sensor_manager
{

/**
 * @brief The SensorManagerServers contains all Sensor Manager related ROS services.
 */
class SensorManagerServers : public temoto_core::BaseSubsystem
{
public:
  /**
   * @brief Constructor.
   * @param b pointer to parent class (each subsystem in TeMoto inherits the base subsystem class).
   * @param sid pointer to Sensor Info Database.
   */
  SensorManagerServers( temoto_core::BaseSubsystem* b, SensorInfoRegistry* sid );

  /**
   * @brief ~SensorManagerServers
   */
  ~SensorManagerServers();

  /**
   * @brief getName
   * @return Name of this subsystem.
   */
  const std::string& getName() const
  {
    return subsystem_name_;
  }
    
private:

  // TODO: Unused service, should be removed
  bool listDevicesCb(temoto_sensor_manager::ListDevices::Request& req, temoto_sensor_manager::ListDevices::Response& res);

  /**
   * @brief Callback to a service that executes/runs a requested device
   * and sends back the topic that the device is publishing to
   * @param req
   * @param res
   */
  void loadSensorCb(temoto_sensor_manager::LoadSensor::Request& req, temoto_sensor_manager::LoadSensor::Response& res);

  /**
   * @brief Called when a sensor is unloaded.
   * @param req
   * @param res
   */
  void unloadSensorCb(temoto_sensor_manager::LoadSensor::Request& req, temoto_sensor_manager::LoadSensor::Response& res);

  /**
   * @brief Called when sensor status update information is received.
   * @param srv
   */
  void statusCb(temoto_core::ResourceStatus& srv);

  /**
   * @brief A function that helps to manage sensor topic related information.
   * @param req_topics Topics that were requested.
   * @param res_topics Response part for the requested topics.
   * @param load_er_msg If topic name remapping is required, then the remapped topic names
   * are placed into this message structure (used later for invoking the sensor via External
   * Resource Manager)
   * @param sensor_info Data structure that contains information about the particular sensor.
   * @param direction Specifies whether input or output topics are managed.
   */
  void processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
                    , std::vector<diagnostic_msgs::KeyValue>& res_topics
                    , temoto_er_manager::LoadExtResource& load_er_msg
                    , SensorInfo& sensor_info
                    , std::string direction);

  /// Pointer to a central Sensor Info Registry object.
  SensorInfoRegistry* sir_;

  /// Resource Management object which handles resource requests and status info propagation.
  temoto_core::rmp::ResourceManager<SensorManagerServers> resource_manager_;

  /// List of allocated sensors.
  std::map<temoto_core::temoto_id::ID, SensorInfo> allocated_sensors_;

}; // SensorManagerServers

} // sensor_manager namespace

#endif
