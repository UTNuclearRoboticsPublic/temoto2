#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensor_manager/sensor_info.h"
#include "sensor_manager/sensor_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include "temoto_2/SensorInfoSync.h"

namespace sensor_manager
{


class SensorManager
{
public:
    /**
     * @brief SensorManager
     */
    SensorManager();

    /**
     * @brief ~SensorManager
     */
    ~SensorManager();

    const std::string& getName() const
    {
      return log_subsys_;
    }
    
  /**
   * \brief Add a new sensor to this manager
   * \param sensor_info Object specifying information about the sensor that is added to this manager.
   */
  void addSensor(const SensorInfo& sensor_info);

private:
  bool indexSensors(const std::string& yaml_filename);
  /**
   * @brief Callback to a service that lists all available packages that
   * are with a requested "type". For example "list all HANDtracking devices"
   * @param req
   * @param res
   * @return
   */
  bool listDevicesCb(temoto_2::ListDevices::Request& req, temoto_2::ListDevices::Response& res);

  void syncCb(const temoto_2::SensorInfoSync& msg);

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
  void startSensorCb(temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

  /**
   * @brief Called when sensor is unloaded. Nothing to do here.
   * @return
   */
  void stopSensorCb(temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

  /**
   * @brief Called when sensor is unloaded. Nothing to do here.
   * @return
   */
  void statusCb(temoto_2::ResourceStatus& srv);

  /**
   * @brief Function for finding the right sensor, based on the request parameters
   * type - requested, name - optional, node - optional
   * @param ret
   * @param retstartSensor
   * @param type
   * @param name
   * @param node
   * @return Returns a boolean. If suitable device was found, then the req param
   * is formatted as a nodeSpawnKill::Request (first one in the list, even if there is more)
   */
  SensorInfoPtr findSensor(std::string type, std::string name, std::string executable);

  std::string log_class_, log_subsys_, log_group_;
  ros::NodeHandle nh_;
  ros::ServiceServer list_devices_server_;
  ros::Publisher sync_pub_;
  ros::Subscriber sync_sub_;
  rmp::ResourceManager<SensorManager> resource_manager_;

  /**
   * @brief List of all defined sensors
   * This list is keep in sync with other sensor managers.
   */
  std::vector<SensorInfoPtr> sensors_;

  /**
   * @brief List of allocated sensors
   */
  std::map<temoto_id::ID, SensorInfoPtr> allocated_sensors_;

}; // SensorManager

} // sensor_manager namespace

#endif
