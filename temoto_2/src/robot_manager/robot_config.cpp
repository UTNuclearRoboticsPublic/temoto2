#include "robot_manager/robot_config.h"
#include "common/tools.h"
#include <string>

namespace robot_manager
{
RobotConfig::RobotConfig(std::string robot_name)
{
  //set the robot to current namespace
  temoto_namespace_ = ::common::getTemotoNamespace();
  robot_name_ = robot_name;
}


std::string RobotConfig::toString() const
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
