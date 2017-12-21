#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "common/temoto_log_macros.h"
#include <yaml-cpp/yaml.h>

typedef std::pair<std::string, std::string> StringPair;

namespace sensor_manager
{

class SensorInfo
{
public:
  /**
   * @brief SensorInfo
   */

  SensorInfo(std::string sensor_name = "A noname sensor");
  
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
    return temoto_namespace_;
  }

  /// Get name
  std::string getName() const
  {
    return sensor_name_;
  }

  std::string getTopic() const
  {
    return topic_;
  }

  // Get output topics
  const std::vector<StringPair>& getTopicsOut() const
  {
    return output_topics_;
  }

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<StringPair>& topics);

  std::string getOutputTopic(const std::string& type);

  // Get sensor type
  std::string getType() const
  {
    return sensor_type_;
  }

  // Get sensor package name
  std::string getPackageName() const
  {
    return package_name_;
  }

  // Get executable
  std::string getExecutable() const
  {
    return executable_;
  }

  // Get description
  std::string getDescription() const
  {
    return description_;
  }

  float getReliability() const
  {
    return reliability_;
  };


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */
  void setTemotoNamespace(std::string temoto_namespace)
  {
    temoto_namespace_ = temoto_namespace;
  }

  void setName(std::string name)
  {
    sensor_name_ = name;
  }

  void setTopic(std::string topic)
  {
    topic_ = topic;
  }

  void addTopicOut(StringPair topic)
  {
    output_topics_.push_back(topic);
  }

  void setType(std::string sensor_type)
  {
    sensor_type_ = sensor_type;
  }

  void setPackageName(std::string package_name)
  {
    package_name_ = package_name;
  }

  void setExecutable(std::string executable)
  {
    executable_ = executable;
  }

  void setDescription(std::string description)
  {
    description_ = description;
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
  
  std::string temoto_namespace_;
  std::string sensor_name_;
  std::string topic_;
  std::string sensor_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  std::vector<StringPair> output_topics_;
  /**
   * @brief Reliability ratings of the sensor.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Reliability moving average.
   */
  float reliability_;

  /**
   * @brief Reliability rating of the sensor.
   */
  unsigned int reliability_idx_;
};

typedef std::shared_ptr<SensorInfo> SensorInfoPtr;
typedef std::vector<SensorInfoPtr> SensorInfoPtrs;

static bool operator==(const SensorInfo& s1, const SensorInfo& s2)
{
  // Check the namespace, executable and name of the package
  if (s1.getTemotoNamespace() != s2.getTemotoNamespace() ||
      s1.getExecutable() != s2.getExecutable() ||
      s1.getPackageName() != s2.getPackageName())
  {
    return false;
  }

  // Check the size of input and output topics
  if (s1.getTopicsOut().size() != s2.getTopicsOut().size())
  {
    return false;
  }

  // Check the output topics
  auto output_topics_2_copy = s2.getTopicsOut();
  for (auto& output_topic_1 : s1.getTopicsOut())
  {
    bool topic_found = false;
    for (auto it=output_topics_2_copy.begin(); it!=output_topics_2_copy.end(); it++)
    {
      if (output_topic_1.first == it->first)
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
    node["executable"] = sensor.getExecutable();
    node["topic"] = sensor.getTopic();
    node["description"] = sensor.getDescription();
    node["reliability"] = sensor.getReliability();

    Node output_topics_node;
    for (auto& topics : sensor.getTopicsOut())
    {
      output_topics_node[topics.first] = topics.second;
    }
    node["output_topics"] = output_topics_node;

    return node;
  }

  static bool decode(const Node& node, sensor_manager::SensorInfo& sensor)
  {
    if (!node.IsMap() || node.size() < 5)
    {
      return false;
    }

    // Convert the compulsory fields
    try
    {
      sensor.setName(node["sensor_name"].as<std::string>());
      sensor.setType(node["sensor_type"].as<std::string>());
      sensor.setPackageName(node["package_name"].as<std::string>());
      sensor.setExecutable(node["executable"].as<std::string>());
      sensor.setTopic(node["topic"].as<std::string>());

      // Get the output_topics
      Node output_topics_node = node["output_topics"];
      for (YAML::const_iterator node_it = output_topics_node.begin(); node_it != output_topics_node.end(); ++node_it)
      {
        sensor.addTopicOut({node_it->first.as<std::string>(), node_it->second.as<std::string>()});
      }
    }
    catch (YAML::InvalidNode e)
    {
      return false;
    }

    // These fields are optional
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
