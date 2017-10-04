#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H


#include "package_info/package_info.h"
#include "sensor_manager/sensor_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"

namespace sensor_manager
{

class SensorManager
{
public:
    /**
     * @brief List of known sensors
     */
    std::vector <package_info> pkgInfoList_;

    /**
     * @brief SensorManager
     */
    SensorManager();

    /**
     * @brief ~SensorManager
     */
    ~SensorManager();

private:

    ros::NodeHandle nh_;
    ros::ServiceServer list_devices_server_;
	rmp::ResourceManager<SensorManager> resource_manager_;

    /**
     * @brief Callback to a service that lists all available packages that
     * are with a requested "type". For example "list all HANDtracking devices"
     * @param req
     * @param res
     * @return
     */
    bool listDevicesCb (temoto_2::ListDevices::Request &req, temoto_2::ListDevices::Response &res);

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
    void startSensorCb (temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

    /**
     * @brief Called when sensor is unloaded. Nothing to do here.
     * @return
     */
    void stopSensorCb (temoto_2::LoadSensor::Request& req, temoto_2::LoadSensor::Response& res);

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
    bool findSensor (temoto_2::LoadProcess::Request& ret,
                     temoto_2::LoadSensor::Response& retstartSensor,
                     std::string type,
                     std::string name,
                     std::string executable);
};

} // sensor_manager namespace

#endif
