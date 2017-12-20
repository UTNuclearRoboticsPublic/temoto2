#ifndef ALGORITHM_INFO_H
#define ALGORITHM_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "common/temoto_log_macros.h"
#include <yaml-cpp/yaml.h>

#include <iostream> // TODO: remove

typedef std::pair<std::string, std::string> StringPair;

namespace algorithm_manager
{

class AlgorithmInfo
{
public:
  /**
   * @brief AlgorithmInfo
   */

  AlgorithmInfo(std::string algorithm_name = "A noname algorithm");
  
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

  // Get the temoto namespace where this algorithm is defined
  std::string getTemotoNamespace() const
  {
    return temoto_namespace_;
  }

  /// Get name
  std::string getName() const
  {
    return algorithm_name_;
  }

  // Get input topics
  const std::vector<StringPair>& getTopicsIn() const
  {
    return input_topics_;
  }

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<StringPair>& topics);

  std::string getInputTopic(const std::string& type);

  std::string getOutputTopic(const std::string& type);

  // Get output topics
  const std::vector<StringPair>& getTopicsOut() const
  {
    return output_topics_;
  }

  // Get algorithm type
  std::string getType() const
  {
    return algorithm_type_;
  }

  // Get algorithm package name
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
    algorithm_name_ = name;
  }

  void addTopicIn(StringPair topic)
  {
    input_topics_.push_back(topic);
  }

  void addTopicOut(StringPair topic)
  {
    output_topics_.push_back(topic);
  }

  void setType(std::string algorithm_type)
  {
    algorithm_type_ = algorithm_type;
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
  
  std::string temoto_namespace_;
  std::string algorithm_name_;
  std::string algorithm_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  std::vector<StringPair> input_topics_;
  std::vector<StringPair> output_topics_;

  /**
   * @brief Reliability ratings of the algorithm.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Reliability moving average.
   */
  float reliability_;

  /**
   * @brief Reliability rating of the algorithm.
   */
  unsigned int reliability_idx_;
};

typedef std::shared_ptr<AlgorithmInfo> AlgorithmInfoPtr;
typedef std::vector<AlgorithmInfoPtr> AlgorithmInfoPtrs;

static bool operator==(const AlgorithmInfo& s1, const AlgorithmInfo& s2)
{
  // Check the namespace, executable and name of the packate
  if (s1.getTemotoNamespace() != s2.getTemotoNamespace() ||
      s1.getExecutable() != s2.getExecutable() ||
      s1.getPackageName() != s2.getPackageName())
  {
    return false;
  }

  // Check the size of input and output topics
  if (s1.getTopicsIn().size() != s2.getTopicsIn().size() ||
      s1.getTopicsOut().size() != s2.getTopicsOut().size())
  {
    return false;
  }

  // Check the input topics
  auto input_topics_2_copy = s2.getTopicsIn();
  for (auto& input_topic_1 : s1.getTopicsIn())
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
    for (auto& topics : algorithm.getTopicsIn())
    {
      input_topics_node[topics.first] = topics.second;
    }
    node["input_topics"] = input_topics_node;

    Node output_topics_node;
    for (auto& topics : algorithm.getTopicsOut())
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
      algorithm.setReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};
}
#endif
