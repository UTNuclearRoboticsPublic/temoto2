#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "temoto_2/SensorInfoSync.h"
#include "common/temoto_log_macros.h"
#include <yaml-cpp/yaml.h>

namespace sensor_manager
{

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


  std::string toString() const;

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

  /**
   * \brief Set reliability
   * \param reliability Sets the initial values for the reliability moving average filter.
   * The value has to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void setReliability(float reliability = 0.8);

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
} // namespace sensor_manager

namespace YAML
{
template <>
struct convert<sensor_manager::SensorInfo>
{
  static Node encode(const sensor_manager::SensorInfo& sensor)
  {
    Node node;
    node["sensor_name"] = sensor.getName();
    node["sensor_type"] = sensor.getType();
    node["package_name"] = sensor.getPackageName();
    node["execute"] = sensor.getExecutable();
    node["topic"] = sensor.getTopic();
    node["description"] = sensor.getDescription();
    node["reliability"] = sensor.getReliability();
    return node;
  }

  static bool decode(const Node& node, sensor_manager::SensorInfo& sensor)
  {
    if (!node.IsMap() || node.size() < 5)
    {
      return false;
    }

    try
    {
      sensor.setName(node["sensor_name"].as<std::string>());
      sensor.setType(node["sensor_type"].as<std::string>());
      sensor.setPackageName(node["package_name"].as<std::string>());
      sensor.setExecutable(node["executable"].as<std::string>());
      sensor.setTopic(node["topic"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
      return false;
    }

    try
    {
      sensor.setDescription(node["description"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    try
    {
      sensor.setReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};
}
#endif
