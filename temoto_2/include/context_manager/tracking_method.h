#ifndef ALGORITHM_INFO_H
#define ALGORITHM_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>                     // shared_ptr
#include "common/temoto_log_macros.h"
#include "common/topic_container.h"   // StringPair
#include "common/reliability.h"
#include <yaml-cpp/yaml.h>

#include <iostream>                   // TODO: remove

namespace algorithm_manager
{

/**
 * @brief The FilterCategory enum
 */
enum class FilterCategory : int
{
  SENSOR,
  ALGORITHM
};

/**
 * @brief The Filter struct
 */
struct Filter
{
  FilterCategory filter_category_;
  std::string filter_type_;
  std::vector<std::string> output_topic_types;
};

/**
 * @brief The TrackerInfo class
 */
class TrackerInfo
{
public:

  /*
   * TODO STUFF
   */

private:
  
  std::string type_;
  std::vector<Filter> pipe_;
  std::string temoto_namespace_;
  Reliability reliability_;
};

typedef std::shared_ptr<TrackerInfo> TrackerInfoPtr;
typedef std::vector<TrackerInfoPtr> TrackerInfoPtrs;

static bool operator==(const TrackerInfo& s1, const TrackerInfo& s2)
{
  // Check the namespace, executable and name of the package
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
struct convert<algorithm_manager::TrackerInfo>
{
  static Node encode(const algorithm_manager::TrackerInfo& algorithm)
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

  static bool decode(const Node& node, algorithm_manager::TrackerInfo& algorithm)
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
