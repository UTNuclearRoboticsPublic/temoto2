#include "sensor_manager/sensor_info.h"
#include "ros/ros.h"
#include "common/tools.h"

namespace sensor_manager
{
SensorInfo::SensorInfo(std::string sensor_name)
{
  //set the sensor to current namespace
  temoto_namespace_ = ::common::getTemotoNamespace();
  sensor_name_ = sensor_name;
}

/* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

// Get the temoto namespace where this sensor is defined
std::string SensorInfo::getTemotoNamespace() const
{
  return temoto_namespace_;
}

/// Get name
std::string SensorInfo::getName() const
{
  return sensor_name_;
}

// Get input topics
const std::vector<StringPair>& SensorInfo::getInputTopics() const
{
  return input_topics_;
}

// Get output topics
const std::vector<StringPair>& SensorInfo::getOutputTopics() const
{
  return output_topics_;
}

// Get topic by type
std::string SensorInfo::getTopicByType(const std::string& type, const std::vector<StringPair>& topics)
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
std::string SensorInfo::getInputTopic(const std::string& type)
{
  return getTopicByType(type, input_topics_);
}

// Get output topic
std::string SensorInfo::getOutputTopic(const std::string& type)
{
  return getTopicByType(type, output_topics_);
}

// Get sensor type
std::string SensorInfo::getType() const
{
  return sensor_type_;
}

// Get sensor package name
std::string SensorInfo::getPackageName() const
{
  return package_name_;
}

// Get executable
std::string SensorInfo::getExecutable() const
{
  return executable_;
}

// Get description
std::string SensorInfo::getDescription() const
{
  return description_;
}

// Get reliability
float SensorInfo::getReliability() const
{
  return reliability_.getReliability();
}

// Is local
bool SensorInfo::isLocal() const
{
  return getTemotoNamespace() == common::getTemotoNamespace();
}

// Get advertised
bool SensorInfo::getAdvertised() const
{
  return advertised_;
}

// To string
std::string SensorInfo::toString() const
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

void SensorInfo::setTemotoNamespace(std::string temoto_namespace)
{
  temoto_namespace_ = temoto_namespace;
}

void SensorInfo::setName(std::string name)
{
  sensor_name_ = name;
}

void SensorInfo::addTopicIn(StringPair topic)
{
  input_topics_.push_back(topic);
}

void SensorInfo::addTopicOut(StringPair topic)
{
  output_topics_.push_back(topic);
}

void SensorInfo::setType(std::string sensor_type)
{
  sensor_type_ = sensor_type;
}

void SensorInfo::setPackageName(std::string package_name)
{
  package_name_ = package_name;
}

void SensorInfo::setExecutable(std::string executable)
{
  executable_ = executable;
}

void SensorInfo::setDescription(std::string description)
{
  description_ = description;
}

void SensorInfo::setAdvertised(bool advertised)
{
  advertised_ = advertised;
}

void SensorInfo::adjustReliability(float reliability)
{
  reliability_.adjustReliability(reliability);
}

void SensorInfo::resetReliability(float reliability)
{
  reliability_.resetReliability(reliability);
}

}  // SensorManager namespace
