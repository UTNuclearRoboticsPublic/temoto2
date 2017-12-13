#include "robot_manager/robot_config.h"
#include "common/tools.h"
#include <string>

namespace robot_manager
{
RobotConfig::RobotConfig(YAML::Node yaml_config) : yaml_config_(yaml_config)
{
  // set the robot to current namespace
  temoto_namespace_ = ::common::getTemotoNamespace();
  try
  {
    getName();
    getPackageName();
    getExecutable();
  }
  catch (...)
  {
    TEMOTO_ERROR("Unable to create robot config: missing robot_name, package_name or executable.");
  }
}

std::string RobotConfig::getName() const
{
  try
  {
    return yaml_config_["robot_name"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: robot_name NOT FOUND");
  }
  return "Unknown robot X";
}

std::string RobotConfig::getPackageName() const
{
  try
  {
    return yaml_config_["package_name"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: package_name NOT FOUND");
  }
  return "unknown_ros_package";
}

std::string RobotConfig::getExecutable() const
{
  try
  {
    return yaml_config_["executable"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: executable NOT FOUND");
  }
  return "unknown_ros_executable";
}

std::string RobotConfig::getDescription() const
{
  try
  {
    return yaml_config_["executable"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: executable NOT FOUND");
  }
  return "unknown_ros_executable";
}

std::string RobotConfig::getUrdfPath() const
{
  std::string urdf_path;
  try
  {
    urdf_path = yaml_config_["urdf"]["executable"].as<std::string>();

    // Ingore pkg_name when executable is defined using absolute path (starts with /)
    if (urdf_path.size() && urdf_path.front() != '/')
    {
      std::string urdf_pkg = yaml_config_["urdf"]["package_name"].as<std::string>();
      urdf_path = urdf_pkg + '/' + urdf_path;
    }
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: urdf:{package_name or executable} NOT FOUND");
    urdf_path = "unknown_ros_executable";
  }
  return urdf_path;
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
