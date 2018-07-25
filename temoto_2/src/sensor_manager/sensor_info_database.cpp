#include "sensor_manager/sensor_info_database.h"

SensorInfoDatabase::SensorInfoDatabase(){}

SensorInfoPtr SensorInfoDatabase::findLocalSensor(temoto_2::LoadSensor::Request& req)
{
  return findSensor(req, local_sensors_);
}

SensorInfoPtr SensorInfoDatabase::findRemoteSensor(temoto_2::LoadSensor::Request& req)
{
  return findSensor(req, remote_sensors_);
}

SensorInfoPtr SensorInfoDatabase::findSensor(temoto_2::LoadSensor::Request& req,
                                             const std::vector<SensorInfoPtr>& sensors)
{
  // Local list of devices that follow the requirements
  std::vector<SensorInfoPtr> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(sensor_infos.begin()
                       , sensor_infos.end()
                       , std::back_inserter(candidates)
                       , [&](const SensorInfoPtr& s)
                         {
                           return s->getType() == req.sensor_type;
                         });

  // The requested type of sensor is not available
  if (candidates.empty())
  {
    return NULL;
  }

  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(),
                            [&](SensorInfoPtr s)
                            {
                              return s->getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfoPtr s)
                            {
                              return s->getExecutable() != req.executable;
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](SensorInfoPtr s)
                            {
                              if (s->getOutputTopics().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s->getOutputTopics();

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
  std::sort(candidates.begin(), it_end, [](SensorInfoPtr& s1, SensorInfoPtr& s2) {
    return s1->getReliability() > s2->getReliability();
  });

  if (candidates.begin() == it_end)
  {
    // Sensor with the requested criteria was not found.
    return NULL;
  }

  // Return the first sensor of the requested type.
  return candidates.front();
}
