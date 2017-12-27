#include "sensor_manager/sensor_info.h"
#include "ros/ros.h"
#include "common/tools.h"

namespace sensor_manager
{
SensorInfo::SensorInfo(std::string sensor_name)
{
  setReliability(0.8);

  //set the sensor to current namespace
  temoto_namespace_ = ::common::getTemotoNamespace();
  sensor_name_ = sensor_name;
}

// Adds a reliability contribution to a moving average filter
void SensorInfo::adjustReliability(float reliability)
{
  reliability = std::max(std::min(reliability, 1.0f), 0.0f);  // clamp to [0-1]
  ++reliability_idx_ %= reliabilities_.size();                 // rolling index

  // take out the last reliability
  reliability_ -= reliabilities_[reliability_idx_] / (float)reliabilities_.size();

  // insert new reliability value
  reliabilities_[reliability_idx_] = reliability;
  reliability_ += reliability / (float)reliabilities_.size();
}

void SensorInfo::setReliability(float reliability)
{
  // Fill array with initial reliability values;
  reliabilities_.fill(reliability);  // buffer for instantaneous reliability values
  reliability_ = reliability;    // Filtered reliability is kept here
  reliability_idx_ = 0;
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
  ret += "  topic            : " + getTopic() + "\n";
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
