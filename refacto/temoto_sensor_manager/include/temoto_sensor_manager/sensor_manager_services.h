#ifndef TEMOTO_SENSOR_MANAGER__SENSOR_MANAGER_SERVICES_H
#define TEMOTO_SENSOR_MANAGER__SENSOR_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_sensor_manager/ListDevices.h"
#include "temoto_sensor_manager/LoadSensor.h"

namespace temoto_sensor_manager
{
  // TODO: Change the srv_name to something more reasonable
  namespace srv_name
  {
      const std::string MANAGER = "sensor_manager";
      const std::string SERVER = "start_sensor";
      const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync";
  }
}

/**
 * @brief operator ==
 * @param r1
 * @param r2
 * @return
 */
static bool operator==(const temoto_sensor_manager::LoadSensor::Request& r1,
                       const temoto_sensor_manager::LoadSensor::Request& r2)
{
  // First check the basics
  if (r1.sensor_type != r2.sensor_type ||
      r1.package_name != r2.package_name ||
      r1.executable != r2.executable)
  {
    return false;
  }

  // Check the size of the requested output topics
  if (r1.output_topics.size() != r2.output_topics.size())
  {
    return false;
  }

  // Check the requested output topics
  auto output_topics_2_copy = r2.output_topics;
  for (auto& output_topic_1 : r1.output_topics)
  {
    bool topic_found = false;
    for (auto it=output_topics_2_copy.begin(); it!=output_topics_2_copy.end(); it++)
    {
      if (output_topic_1.key == it->key &&
          output_topic_1.value == it->value)
      {
        topic_found = true;
        output_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
  }

  // The sensor infos are equal
  return true;
}
#endif
