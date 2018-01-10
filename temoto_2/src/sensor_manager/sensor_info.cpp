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
std::string SensorInfo::getOutputTopic(const std::string& type)
{
  return getTopicByType(type, output_topics_);
}

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

}  // SensorManager namespace
