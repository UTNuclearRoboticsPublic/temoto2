#include "sensor_manager/sensor_info_registry.h"
#include <algorithm>

namespace sensor_manager
{

SensorInfoRegistry::SensorInfoRegistry(){}

bool SensorInfoRegistry::addLocalSensor(const SensorInfo& si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such sensor
  SensorInfo si_ret;
  if (!findSensor(si, local_sensors_, si_ret))
  {
    local_sensors_.push_back(si);
    return true;
  }

  // Return false if such sensor already exists
  return false;
}

bool SensorInfoRegistry::addRemoteSensor(const SensorInfo &si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such sensor
  SensorInfo si_ret;
  if (!findSensor(si, remote_sensors_, si_ret))
  {
    remote_sensors_.push_back(si);
    return true;
  }

  // Return false if such sensor already exists
  return false;
}

bool SensorInfoRegistry::updateLocalSensor(const SensorInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( local_sensors_.begin()
                              , local_sensors_.end()
                              , [&](const SensorInfo& ls)
                              {
                                return ls == si;
                              });

  // Update the local sensor if its found
  if (it != local_sensors_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such sensor was found
  return false;
}

bool SensorInfoRegistry::updateRemoteSensor(const SensorInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( remote_sensors_.begin()
                              , remote_sensors_.end()
                              , [&](const SensorInfo& rs)
                              {
                                return rs == si;
                              });

  // Update the local sensor if its found
  if (it != remote_sensors_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such sensor was found
  return false;
}

bool SensorInfoRegistry::findLocalSensors( temoto_2::LoadSensor::Request& req
                                         , std::vector<SensorInfo>& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findSensors(req, local_sensors_, si_ret);
}

bool SensorInfoRegistry::findLocalSensor( const SensorInfo &si, SensorInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findSensor(si, local_sensors_, si_ret);
}

bool SensorInfoRegistry::findLocalSensor( const SensorInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  SensorInfo si_ret;
  return findSensor(si, local_sensors_, si_ret);
}

bool SensorInfoRegistry::findRemoteSensors( temoto_2::LoadSensor::Request& req
                                          , std::vector<SensorInfo>& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findSensors(req, remote_sensors_, si_ret);
}

bool SensorInfoRegistry::findRemoteSensor( const SensorInfo &si, SensorInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findSensor(si, remote_sensors_, si_ret);
}

bool SensorInfoRegistry::findRemoteSensor( const SensorInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  SensorInfo si_ret;
  return findSensor(si, remote_sensors_, si_ret);
}

bool SensorInfoRegistry::findSensors( temoto_2::LoadSensor::Request& req
                                    , const std::vector<SensorInfo>& sensors
                                    , std::vector<SensorInfo>& si_ret ) const
{
  // Local list of devices that follow the requirements
  std::vector<SensorInfo> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(sensors.begin()
                       , sensors.end()
                       , std::back_inserter(candidates)
                       , [&](const SensorInfo& s)
                         {
                           return s.getType() == req.sensor_type;
                         });

  // The requested type of sensor is not available
  if (candidates.empty())
  {
    return false;
  }

  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(),
                            [&](SensorInfo s)
                            {
                              return s.getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfo s)
                            {
                              return s.getExecutable() != req.executable;
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfo s)
                            {
                              if (s.getOutputTopics().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s.getOutputTopics();

                              // Start looking for the requested topic types
                              for (auto& topic : req.output_topics)
                              {
                                bool found = false;
                                for (auto it=output_topics_copy.begin(); it != output_topics_copy.end(); it++)
                                {
                                  // If the topic was found then remove it from the copy list
                                  if (topic.key == it->first)
                                  {
                                    found = true;
                                    output_topics_copy.erase(it);
                                    break;
                                  }
                                }

                                // If this topic type was not found then return with false
                                if (!found)
                                {
                                  return true;
                                }
                              }

                              return false;
                            });
  }

  // Sort remaining candidates based on their reliability.
  std::sort( candidates.begin()
           , it_end
           , [](SensorInfo& s1, SensorInfo& s2)
             {
               return s1.getReliability() > s2.getReliability();
             });

  if (candidates.begin() == it_end)
  {
    // Sensor with the requested criteria was not found.
    return false;
  }

  // Return the first sensor of the requested type.
  si_ret = candidates;
  return true;
}

bool SensorInfoRegistry::findSensor( const SensorInfo &si
                                   , const std::vector<SensorInfo>& sensors
                                   , SensorInfo& si_ret ) const
{

  const auto it = std::find_if( sensors.begin()
                              , sensors.end()
                              , [&](const SensorInfo& rs)
                              {
                                return rs == si;
                              });


  if (it == sensors.end())
  {
    return false;
  }
  else
  {
    si_ret = *it;
    return true;
  }
}

const std::vector<SensorInfo>& SensorInfoRegistry::getLocalSensors() const
{
  return local_sensors_;
}

const std::vector<SensorInfo>& SensorInfoRegistry::getRemoteSensors() const
{
  return remote_sensors_;
}

} // sensor_manager namespace
