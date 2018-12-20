#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/topic_container.h"   // StringPair
#include "temoto_core/common/reliability.h"
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
   * @param sensor_name
   */
  SensorInfo(std::string sensor_name = "A noname sensor");
  
  // To string
  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this sensor is defined
  std::string getTemotoNamespace() const;

  /// Get name
  std::string getName() const;

  // Get input topics
  const std::vector<StringPair>& getInputTopics() const;

  // Get output topics
  const std::vector<StringPair>& getOutputTopics() const;

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<StringPair>& topics);

  // Get input topic
  std::string getInputTopic(const std::string& type);

  // Get output topic
  std::string getOutputTopic(const std::string& type);

  // Get sensor type
  std::string getType() const;

  // Get sensor package name
  std::string getPackageName() const;

  // Get executable
  std::string getExecutable() const;

  // Get description
  std::string getDescription() const;

  // Get reliability
  float getReliability() const;

  // Is local
  bool isLocal() const;

  // Get advertised
  bool getAdvertised() const;


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */

  void setTemotoNamespace(std::string temoto_namespace);

  void setName(std::string name);

  void addTopicIn(StringPair topic);

  void addTopicOut(StringPair topic);

  void setType(std::string sensor_type);

  void setPackageName(std::string package_name);

  void setExecutable(std::string executable);

  void setDescription(std::string description);

  void setAdvertised(bool advertised);

  void adjustReliability(float reliability);

  void resetReliability(float reliability);


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
  temoto_core::Reliability reliability_;
  std::vector<StringPair> input_topics_;
  std::vector<StringPair> output_topics_;
  bool advertised_ = false;
};

typedef std::shared_ptr<SensorInfo> SensorInfoPtr;
typedef std::vector<SensorInfoPtr> SensorInfoPtrs;

// TODO: Not sure how to declare operators in a header, implement them in src
//       and not brake everything (linking problems in places where
//       sensor_info.cpp has to be linked)
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
  if (s1.getInputTopics().size() != s2.getInputTopics().size() ||
      s1.getOutputTopics().size() != s2.getOutputTopics().size())
  {
    return false;
  }

  // Check the input topics
  auto input_topics_2_copy = s2.getInputTopics();
  for (auto& input_topic_1 : s1.getInputTopics())
  {
    bool topic_found = false;
    for (auto it=input_topics_2_copy.begin(); it!=input_topics_2_copy.end(); it++)
    {
      if (input_topic_1.first == it->first)
      {
        topic_found = true;
        input_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
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

    Node input_topics_node;
    for (auto& topics : sensor.getInputTopics())
    {
      input_topics_node[topics.first] = topics.second;
    }
    node["input_topics"] = input_topics_node;

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

    uint8_t code;

    // Convert the compulsory fields
    try
    {
      code = 1;
      sensor.setName(node["sensor_name"].as<std::string>());

      code = 2;
      sensor.setType(node["sensor_type"].as<std::string>());

      code = 3;
      sensor.setPackageName(node["package_name"].as<std::string>());

      code = 4;
      sensor.setExecutable(node["executable"].as<std::string>());

      // Get the output_topics
      code = 6;
      Node output_topics_node = node["output_topics"];
      for (YAML::const_iterator node_it = output_topics_node.begin(); node_it != output_topics_node.end(); ++node_it)
      {
        sensor.addTopicOut({node_it->first.as<std::string>(),
                               node_it->second.as<std::string>()});
      }

    }
    catch (YAML::InvalidNode e)
    {
      // print out the error message
      switch(code)
      {
        case 5:
          std::cout << "Something is wrong with the 'input_topics'\n";
          break;

        case 6:
          std::cout << "Something is wrong with the 'output_topics'\n";
          break;
      }

      return false;
    }

    /*
     * These fields are optional
     */

    // Get the input_topics
    try
    {
      Node input_topics_node = node["input_topics"];
      for (YAML::const_iterator node_it = input_topics_node.begin()
          ; node_it != input_topics_node.end()
          ; ++node_it)
      {
        sensor.addTopicIn({node_it->first.as<std::string>(), node_it->second.as<std::string>()});
      }
    }
    catch (YAML::InvalidNode e)
    {
    }

    // Get the description
    try
    {
      sensor.setDescription(node["description"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    // Get the reliability
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
