#include "algorithm_manager/algorithm_info.h"
#include "ros/ros.h"
#include "temoto_core/common/tools.h"

namespace algorithm_manager
{
AlgorithmInfo::AlgorithmInfo(std::string algorithm_name)
{
  //set the algorithm to current namespace
  temoto_namespace_ = ::temoto_core::common::getTemotoNamespace();
  algorithm_name_ = algorithm_name;
}

/* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

// Get the temoto namespace where this algorithm is defined
std::string AlgorithmInfo::getTemotoNamespace() const
{
  return temoto_namespace_;
}

/// Get name
std::string AlgorithmInfo::getName() const
{
  return algorithm_name_;
}

// Get input topics
const std::vector<temoto_core::StringPair>& AlgorithmInfo::getInputTopics() const
{
  return input_topics_;
}

// Get output topics
const std::vector<temoto_core::StringPair>& AlgorithmInfo::getOutputTopics() const
{
  return output_topics_;
}

// Get topic by type
std::string AlgorithmInfo::getTopicByType(const std::string& type, const std::vector<temoto_core::StringPair>& topics)
{
  // Loop over the topics and check the type match. Return the topic if the types amatch
  for(auto&topic : topics)
  {
    if(topic.first == type)
    {
      return topic.second;
    }
  }

  return std::string();
}

// Get output topic
std::string AlgorithmInfo::getInputTopic(const std::string& type)
{
  return getTopicByType(type, input_topics_);
}

// Get output topic
std::string AlgorithmInfo::getOutputTopic(const std::string& type)
{
  return getTopicByType(type, output_topics_);
}

// Get algorithm type
std::string AlgorithmInfo::getType() const
{
  return algorithm_type_;
}

// Get algorithm package name
std::string AlgorithmInfo::getPackageName() const
{
  return package_name_;
}

// Get executable
std::string AlgorithmInfo::getExecutable() const
{
  return executable_;
}

// Get description
std::string AlgorithmInfo::getDescription() const
{
  return description_;
}

// Get reliability
float AlgorithmInfo::getReliability() const
{
  return reliability_.getReliability();
}

// Is local
bool AlgorithmInfo::isLocal() const
{
  return getTemotoNamespace() == temoto_core::common::getTemotoNamespace();
}

// Get advertised
bool AlgorithmInfo::getAdvertised() const
{
  return advertised_;
}

// To string
std::string AlgorithmInfo::toString() const
{
  std::string ret;
  ret += "SENSOR: " + getName() + "\n";
  ret += "  temoto_namespace : " + getTemotoNamespace() + "\n";
  ret += "  type             : " + getType() + "\n";
  ret += "  package name     : " + getPackageName() + "\n";
  ret += "  executable       : " + getExecutable() + "\n";
  ret += "  description      : " + getDescription() + "\n";
  ret += "  reliability      : " + std::to_string(getReliability()) + "\n";

  // Print out the input topics
  if (!getInputTopics().empty())
  {
    ret += "  output_topics \n";
    for (auto& topic : getInputTopics())
    {
      ret += "    " + topic.first + " : " + topic.second + "\n";
    }
  }

  // Print out the output topics
  if (!getOutputTopics().empty())
  {
    ret += "  output_topics \n";
    for (auto& topic : getOutputTopics())
    {
      ret += "    " + topic.first + " : " + topic.second + "\n";
    }
  }

  return ret;
}

/* * * * * * * * * * * *
 *     SETTERS
 * * * * * * * * * * * */

void AlgorithmInfo::setTemotoNamespace(std::string temoto_namespace)
{
  temoto_namespace_ = temoto_namespace;
}

void AlgorithmInfo::setName(std::string name)
{
  algorithm_name_ = name;
}

void AlgorithmInfo::addTopicIn(temoto_core::StringPair topic)
{
  input_topics_.push_back(topic);
}

void AlgorithmInfo::addTopicOut(temoto_core::StringPair topic)
{
  output_topics_.push_back(topic);
}

void AlgorithmInfo::setType(std::string algorithm_type)
{
  algorithm_type_ = algorithm_type;
}

void AlgorithmInfo::setPackageName(std::string package_name)
{
  package_name_ = package_name;
}

void AlgorithmInfo::setExecutable(std::string executable)
{
  executable_ = executable;
}

void AlgorithmInfo::setDescription(std::string description)
{
  description_ = description;
}

void AlgorithmInfo::setAdvertised(bool advertised)
{
  advertised_ = advertised;
}

void AlgorithmInfo::adjustReliability(float reliability)
{
  reliability_.adjustReliability(reliability);
}

void AlgorithmInfo::resetReliability(float reliability)
{
  reliability_.resetReliability(reliability);
}

}  // AlgorithmManager namespace
