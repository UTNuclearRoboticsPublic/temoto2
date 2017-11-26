#include "robot_manager/robot_info.h"
#include "ros/ros.h"
#include "common/tools.h"

namespace robot_manager
{
RobotInfo::RobotInfo(std::string robot_name)
{
  setReliability(0.8);

  //set the robot to current namespace
  temoto_namespace_ = ::common::getTemotoNamespace();
  robot_name_ = robot_name;
}

// Adds a reliability contribution to a moving average filter
void RobotInfo::adjustReliability(float reliability)
{
  reliability = std::max(std::min(reliability, 1.0f), 0.0f);  // clamp to [0-1]
  ++reliability_idx_ %= reliabilities_.size();                 // rolling index

  // take out the last reliability
  reliability_ -= reliabilities_[reliability_idx_] / (float)reliabilities_.size();

  // insert new reliability value
  reliabilities_[reliability_idx_] = reliability;
  reliability_ += reliability / (float)reliabilities_.size();
}

void RobotInfo::setReliability(float reliability)
{
  // Fill array with initial reliability values;
  reliabilities_.fill(reliability);  // buffer for instantaneous reliability values
  reliability_ = reliability;    // Filtered reliability is kept here
  reliability_idx_ = 0;
}

std::string RobotInfo::toString() const
{
  std::string ret;
  ret += "ROBOT: " + getName() + "\n";
  ret += "  package name: " + getPackageName() + "\n";
  ret += "  executable  : " + getExecutable() + "\n";
  ret += "  description : " + getDescription() + "\n";
  ret += "  reliability : " + std::to_string(getReliability()) + "\n";
  return ret;
}

}  // RobotManager namespace
