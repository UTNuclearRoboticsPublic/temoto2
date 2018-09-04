#ifndef SENSOR_MANAGER_SERVERS_H
#define SENSOR_MANAGER_SERVERS_H

#include "common/base_subsystem.h"
#include "sensor_manager/sensor_info_registry.h"
#include "sensor_manager/sensor_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"

#include "std_msgs/String.h"
#include "common/temoto_id.h"

namespace sensor_manager
{

typedef std_msgs::String PayloadType;

class SensorManagerServers : public BaseSubsystem
{
public:
  /**
   * @brief SensorManagerServers
   */
  SensorManagerServers( BaseSubsystem* b, SensorInfoRegistry* sid );

  /**
   * @brief ~SensorManagerServers
   */
  ~SensorManagerServers();

  const std::string& getName() const
  {
    return subsystem_name_;
  }
    
private:
  //bool indexSensors(const std::string& yaml_filename);
  /**
   * @brief Callback to a service that lists all available packages that
   * are with a requested "type". For example "list all HANDtracking devices"
   * @param req
   * @param res
   * @return
   */
  bool listDevicesCb(temoto_2::ListDevices::Request& req, temoto_2::ListDevices::Response& res);

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
  void loadSensorCb(temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

  /**
   * @brief Called when sensor is unloaded. Nothing to do here.
   * @return
   */
  void unloadSensorCb(temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

  /**
   * @brief Called when sensor is unloaded. Nothing to do here.
   * @return
   */
  void statusCb(temoto_2::ResourceStatus& srv);

  /**
   * @brief processTopics
   * @param req
   * @param res
   * @param load_process_msg
   * @param algorithm_ptr
   */
  void processTopics(std::vector<diagnostic_msgs::KeyValue>& req_topics
                    , std::vector<diagnostic_msgs::KeyValue>& res_topics
                    , temoto_2::LoadProcess& load_process_msg
                    , SensorInfo& sensor_info
                    , bool inputTopics);

  /// Sensor Info Registry
  SensorInfoRegistry* sir_;

  ///  ros::ServiceServer list_devices_server_;
  rmp::ResourceManager<SensorManagerServers> resource_manager_;

  /// List of allocated sensors
  std::map<temoto_id::ID, SensorInfo> allocated_sensors_;

}; // SensorManagerServers

} // sensor_manager namespace

#endif
