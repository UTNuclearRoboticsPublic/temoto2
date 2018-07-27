#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include "common/temoto_log_macros.h"
#include "common/topic_container.h"   // StringPair
#include "common/reliability.h"
#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>                     // shared_ptr
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
  
  void adjustReliability(float reliability)
  {
    reliability_.adjustReliability(reliability);
  }

  void resetReliability(float reliability)
  {
    reliability_.resetReliability(reliability);
  }


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

  // Get output topics
  const std::vector<StringPair>& getOutputTopics() const
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
    return reliability_.getReliability();
  }

  bool isLocal() const
  {
    return getTemotoNamespace() == common::getTemotoNamespace();
  }

  bool getUpdated() const
  {
    return updated_;
  }


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

  void setUpdated(bool updated)
  {
    updated_ = updated;
  }


private:

  std::string log_class_ = "SensorInfo";
  std::string log_subsys_ = "sensor_manager";
  std::string log_group_ = "sensor_manager";
  
  std::string temoto_namespace_;
  std::string sensor_name_;
  std::string sensor_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  Reliability reliability_;
  std::vector<StringPair> output_topics_;
  bool updated_ = false;
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
  if (s1.getOutputTopics().size() != s2.getOutputTopics().size())
  {
    return false;
  }

  // Check the output topics
  auto output_topics_2_copy = s2.getOutputTopics();
  for (auto& output_topic_1 : s1.getOutputTopics())
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
    node["description"] = sensor.getDescription();
    node["reliability"] = sensor.getReliability();

    Node output_topics_node;
    for (auto& topics : sensor.getOutputTopics())
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
      sensor.resetReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};
}
#endif
