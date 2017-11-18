#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "temoto_2/SensorInfoSync.h"
#include "common/temoto_log_macros.h"

namespace sensor_manager
{
namespace sync_action
{
//const std::string ADD = "add";
const std::string UPDATE = "update";
const std::string GET_SENSORS = "get_sensors";
}

class SensorInfo
{
public:
  /**
   * @brief SensorInfo
   */

  SensorInfo(std::string sensor_name = "A noname sensor");
  
  SensorInfo(const temoto_2::SensorInfoSync& msg);

  /**
   * \brief Adjust reliability
   * \param reliability the new reliability contribution to the moving average filter. The value has
   * to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void adjustReliability(float reliability = 1.0);

  /**
   * \brief Reset reliability
   * \param initial_reliability Sets the initial values for the reliability moving average filter.
   * The value has to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void resetReliability(float reliability = 0.8);

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this sensor is defined
  std::string getTemotoNamespace() const
  {
    return msg_.temoto_namespace;
  }

  /// Get name
  std::string getName() const
  {
    return msg_.sensor_name;
  }

  std::string getTopic() const
  {
    return msg_.topic;
  }

  // Get sensor type
  std::string getType() const
  {
    return msg_.sensor_type;
  }

  // Get sensor package name
  std::string getPackageName() const
  {
    return msg_.package_name;
  }

  // Get executable
  std::string getExecutable() const
  {
    return msg_.executable;
  }

  // Get description
  std::string getDescription() const
  {
    return msg_.description;
  }

  float getReliability() const
  {
    return msg_.reliability;
  };

  // get sync message with proper sync_action (ADD or UPDATE)
  const temoto_2::SensorInfoSync& getSyncMsg(const std::string& action);


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */
  void setTemotoNamespace(std::string temoto_namespace)
  {
    msg_.temoto_namespace = temoto_namespace;
  }

  void setName(std::string name)
  {
    msg_.sensor_name = name;
  }

  void setTopic(std::string topic)
  {
    msg_.topic = topic;
  }

  void setType(std::string sensor_type)
  {
    msg_.sensor_type = sensor_type;
  }

  void setPackageName(std::string package_name)
  {
    msg_.package_name = package_name;
  }

  void setExecutable(std::string executable)
  {
    msg_.executable = executable;
  }

  void setDescription(std::string description)
  {
    msg_.description = description;
  }

  void setReliability(float reliability)
  {
    msg_.reliability = reliability;
  }

private:

  std::string log_class_ = "SensorInfo";
  std::string log_subsys_ = "sensor_manager";
  std::string log_group_ = "sensor_manager";
  
  /**
   * @brief Message container where the sensor info is stored.
   */
  temoto_2::SensorInfoSync msg_;

  /**
   * @brief Reliability ratings of the sensor.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Reliability moving average.
   */
  float reliability_average;

  /**
   * @brief Reliability rating of the sensor.
   */
  unsigned int reliability_idx;
};

typedef std::shared_ptr<SensorInfo> SensorInfoPtr;

static bool operator==(const SensorInfo& s1, const SensorInfo& s2)
{
  return (s1.getTemotoNamespace() == s2.getTemotoNamespace() && s1.getTopic() == s2.getTopic() &&
          s1.getExecutable() == s2.getExecutable() && s1.getPackageName() == s2.getPackageName());
}
}
#endif
