#ifndef SENSOR_INFO_DATABASE_H
#define SENSOR_INFO_DATABASE_H

#include "sensor_manager/sensor_info.h"
#include "temoto_2/LoadSensor.h"

#include <mutex>

namespace sensor_manager
{

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

  bool findLocalSensor( temoto_2::LoadSensor::Request& req, SensorInfo& si_ret ) const;

  bool findLocalSensor( const SensorInfo& si, SensorInfo& si_ret ) const;

  bool findLocalSensor( const SensorInfo& si ) const;

  bool findRemoteSensor( temoto_2::LoadSensor::Request& req, SensorInfo& si_ret ) const;

  bool findRemoteSensor( const SensorInfo& si, SensorInfo& si_ret ) const;

  bool findRemoteSensor( const SensorInfo& si ) const;

  bool addLocalSensor( const SensorInfo& si );

  bool addRemoteSensor( const SensorInfo& si );

  bool updateLocalSensor(const SensorInfo& si, bool advertised = false);

  bool updateRemoteSensor(const SensorInfo& si, bool advertised = false);

  const std::vector<SensorInfo>& getLocalSensors() const;

  const std::vector<SensorInfo>& getRemoteSensors() const;

private:

  /**
   * @brief findSensor
   * @param req
   * @param sensors
   * @return
   */
  bool findSensor( temoto_2::LoadSensor::Request& req
                 , const std::vector<SensorInfo>& sensors
                 , SensorInfo& si_ret ) const;

  bool findSensor( const SensorInfo& si
                 , const std::vector<SensorInfo>& sensors
                 , SensorInfo& si_ret ) const;

  /// List of all locally defined sensors.
  std::vector<SensorInfo> local_sensors_;

  /// List of all sensors in remote managers.
  std::vector<SensorInfo> remote_sensors_;

  /// Mutex for protecting sensor info vectors from data races
  mutable std::mutex read_write_mutex;
};

} // sensor_manager namespace

#endif
