#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/topic_container.h"   // temoto_core::StringPair
#include "temoto_core/common/reliability.h"
#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>                     // shared_ptr
#include <yaml-cpp/yaml.h>

namespace algorithm_manager
{

class AlgorithmInfo
{
public:

  /**
   * @brief AlgorithmInfo
   * @param algorithm_name
   */
  AlgorithmInfo(std::string algorithm_name = "A noname algorithm");

  // To string
  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this algorithm is defined
  std::string getTemotoNamespace() const;

  /// Get name
  std::string getName() const;

  // Get input topics
  const std::vector<temoto_core::StringPair>& getInputTopics() const;

  // Get output topics
  const std::vector<temoto_core::StringPair>& getOutputTopics() const;

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<temoto_core::StringPair>& topics);

  // Get input topic
  std::string getInputTopic(const std::string& type);

  // Get output topic
  std::string getOutputTopic(const std::string& type);

  // Get algorithm type
  std::string getType() const;

  // Get algorithm package name
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

  void addTopicIn(temoto_core::StringPair topic);

  void addTopicOut(temoto_core::StringPair topic);

  void setType(std::string algorithm_type);

  void setPackageName(std::string package_name);

  void setExecutable(std::string executable);

  void setDescription(std::string description);

  void setAdvertised(bool advertised);

  void adjustReliability(float reliability);

  void resetReliability(float reliability);


private:

  std::string log_class_ = "AlgorithmInfo";
  std::string log_subsys_ = "algorithm_manager";
  std::string log_group_ = "algorithm_manager";

  std::string temoto_namespace_;
  std::string algorithm_name_;
  std::string algorithm_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  temoto_core::Reliability reliability_;
  std::vector<temoto_core::StringPair> input_topics_;
  std::vector<temoto_core::StringPair> output_topics_;
  bool advertised_ = false;
};

typedef std::shared_ptr<AlgorithmInfo> AlgorithmInfoPtr;
typedef std::vector<AlgorithmInfoPtr> AlgorithmInfoPtrs;

// TODO: Not sure how to declare operators in a header, implement them in src
//       and not brake everything (linking problems in places where
//       algorithm_info.cpp has to be linked)
static bool operator==(const AlgorithmInfo& s1, const AlgorithmInfo& s2)
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

  // The algorithm infos are equal
  return true;
}
} // namespace algorithm_manager

namespace YAML
{
template <>
struct convert<algorithm_manager::AlgorithmInfo>
{
  static Node encode(const algorithm_manager::AlgorithmInfo& algorithm)
  {
    Node node;
    node["algorithm_name"] = algorithm.getName();
    node["algorithm_type"] = algorithm.getType();
    node["package_name"] = algorithm.getPackageName();
    node["executable"] = algorithm.getExecutable();
    node["description"] = algorithm.getDescription();
    node["reliability"] = algorithm.getReliability();

    Node input_topics_node;
    for (auto& topics : algorithm.getInputTopics())
    {
      input_topics_node[topics.first] = topics.second;
    }
    node["input_topics"] = input_topics_node;

    Node output_topics_node;
    for (auto& topics : algorithm.getOutputTopics())
    {
      output_topics_node[topics.first] = topics.second;
    }
    node["output_topics"] = output_topics_node;

    return node;
  }

  static bool decode(const Node& node, algorithm_manager::AlgorithmInfo& algorithm)
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
      algorithm.setName(node["algorithm_name"].as<std::string>());

      code = 2;
      algorithm.setType(node["algorithm_type"].as<std::string>());

      code = 3;
      algorithm.setPackageName(node["package_name"].as<std::string>());

      code = 4;
      algorithm.setExecutable(node["executable"].as<std::string>());

      // Get the input_topics
      code = 5;
      Node input_topics_node = node["input_topics"];
      for (YAML::const_iterator node_it = input_topics_node.begin(); node_it != input_topics_node.end(); ++node_it)
      {
        algorithm.addTopicIn({node_it->first.as<std::string>(),
                              node_it->second.as<std::string>()});
      }

      // Get the output_topics
      code = 6;
      Node output_topics_node = node["output_topics"];
      for (YAML::const_iterator node_it = output_topics_node.begin(); node_it != output_topics_node.end(); ++node_it)
      {
        algorithm.addTopicOut({node_it->first.as<std::string>(),
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

    // These fields are optional
    try
    {
      algorithm.setDescription(node["description"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    try
    {
      algorithm.resetReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};
}
#endif
