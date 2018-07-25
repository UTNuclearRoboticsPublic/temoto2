#ifndef SENSOR_INFO_DATABASE_H
#define SENSOR_INFO_DATABASE_H

#include "sensor_manager/sensor_info.h"
#include "temoto_2/LoadSensor.h"

#include <mutex>

/**
 * @brief Class that maintains and handles the sensor info objects
 */
class SensorInfoDatabase
{
public:

  struct SensorInfoPtrs
  {
    std::vector<SensorInfoPtr>& sensors;
  };

  SensorInfoDatabase();

  SensorInfoPtr findLocalSensor(temoto_2::LoadSensor::Request& req);

  SensorInfoPtr findRemoteSensor(temoto_2::LoadSensor::Request& req);

private:

  /**
   * @brief findSensor
   * @param req
   * @param sensors
   * @return
   */
  SensorInfoPtr findSensor(temoto_2::LoadSensor::Request& req, const std::vector<SensorInfoPtr>& sensors);

  /// List of all locally defined sensors.
  std::vector<SensorInfoPtr> local_sensors_;

  /// List of all sensors in remote managers.
  std::vector<SensorInfoPtr> remote_sensors_;

  /// Mutex for protecting sensor info vectors from data races
  std::mutex read_write_mutex;
};

#endif
