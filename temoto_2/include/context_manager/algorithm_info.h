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

namespace context_manager
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

  std::string getTopic() const
  {
    return topic_;
  }

  // Get input topics
  std::vector<StringPair> getTopicsIn() const
  {
    return topics_in_;
  }

  // Get output topics
  std::vector<StringPair> getTopicsOut() const
  {
    return topics_out_;
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

  void setTopic(std::string topic)
  {
    topic_ = topic;
  }

  void addTopicIn(StringPair topic)
  {
    topics_in_.push_back(topic);
  }

  void addTopicOut(StringPair topic)
  {
    topics_out_.push_back(topic);
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

  std::string log_class_ = "AlgorithmInfo";
  std::string log_subsys_ = "context_manager";
  std::string log_group_ = "context_manager";
  
  std::string temoto_namespace_;
  std::string algorithm_name_;
  std::string topic_;
  std::string algorithm_type_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  std::vector<StringPair> topics_in_;
  std::vector<StringPair> topics_out_;

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
  return (s1.getTemotoNamespace() == s2.getTemotoNamespace() && s1.getTopic() == s2.getTopic() &&
          s1.getExecutable() == s2.getExecutable() && s1.getPackageName() == s2.getPackageName());
}
} // namespace context_manager

namespace YAML
{
template <>
struct convert<context_manager::AlgorithmInfo>
{
  static Node encode(const context_manager::AlgorithmInfo& algorithm)
  {
    Node node;
    node["algorithm_name"] = algorithm.getName();
    node["algorithm_type"] = algorithm.getType();
    node["package_name"] = algorithm.getPackageName();
    node["executable"] = algorithm.getExecutable();
    node["topic"] = algorithm.getTopic();
    node["description"] = algorithm.getDescription();
    node["reliability"] = algorithm.getReliability();

    Node topics_in_node;
    for (auto& topics : algorithm.getTopicsIn())
    {
      topics_in_node[topics.first] = topics.second;
    }
    node["topics_in"] = topics_in_node;

    Node topics_out_node;
    for (auto& topics : algorithm.getTopicsOut())
    {
      topics_out_node[topics.first] = topics.second;
    }
    node["topics_out"] = topics_out_node;

    return node;
  }

  static bool decode(const Node& node, context_manager::AlgorithmInfo& algorithm)
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

      code = 5;
      algorithm.setTopic(node["topic"].as<std::string>());

      // Get the topics_in
      code = 6;
      Node topics_in_node = node["topics_in"];
      for (YAML::const_iterator node_it = topics_in_node.begin(); node_it != topics_in_node.end(); ++node_it)
      {
        algorithm.addTopicIn({node_it->first.as<std::string>(),
                              node_it->second.as<std::string>()});
      }

      // Get the topics_out
      code = 7;
      Node topics_out_node = node["topics_out"];
      for (YAML::const_iterator node_it = topics_out_node.begin(); node_it != topics_out_node.end(); ++node_it)
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
        case 6:
          std::cout << "Something is wrong with the 'topics_in'\n";
          break;

        case 7:
          std::cout << "Something is wrong with the 'topics_out'\n";
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
