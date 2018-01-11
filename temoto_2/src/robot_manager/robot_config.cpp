#include "robot_manager/robot_config.h"
#include "common/tools.h"
#include <string>
#include <vector>

namespace robot_manager
{
RobotConfig::RobotConfig(YAML::Node yaml_config, BaseSubsystem& b) : yaml_config_(yaml_config), BaseSubsystem(b)
{
  class_name_ = "RobotConfig";
  // Parse mandatory information.
  try
  {
    parseName();
  }
  catch (...)
  {
    CREATE_ERROR(error::Code::ROBOT_CONFIG_FAIL,"Unable to parse robot name.");
    return; //\TODO: throw and skip the rest when requred info is missing
  }

  // Parse additional information
  parseTemotoNamespace();
  parseDescription();
  parseReliability();

  // Parse robot features
  parseUrdf();
  parseManipulation();
  parseNavigation();
}

void RobotConfig::parseName()
{
  try
  {
    name_ = yaml_config_["robot_name"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_ERROR("CONFIG: robot_name NOT FOUND");
    name_ = "unnamed_robot";
    //\TODO: throw std::string
  }
}


void RobotConfig::parseTemotoNamespace()
{
  try
  {
    temoto_namespace_ = yaml_config_["temoto_namespace"].as<std::string>();
  }
  catch (...)
  {
    // Assign local namespace, when not available in yaml
    temoto_namespace_ = common::getTemotoNamespace();
  }
}

void RobotConfig::parseDescription()
{
  try
  {
    description_ = yaml_config_["description"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_WARN("CONFIG: description NOT FOUND");
  }
}

void RobotConfig::parseReliability()
{
  try
  {
    description_ = yaml_config_["description"].as<std::string>();
  }
  catch (YAML::InvalidNode e)
  {
    TEMOTO_WARN("CONFIG: reliability NOT FOUND");
  }
}

void RobotConfig::parseUrdf()
{
  try
  {
    feature_urdf_ = FeatureURDF(yaml_config_["urdf"]);
  }
  catch (...)
  {
    TEMOTO_ERROR("CONFIG: urdf:{package_name or executable} NOT FOUND");
  }
}

void RobotConfig::parseManipulation()
{
  try
  {
    feature_manipulation_ = FeatureManipulation(yaml_config_["manipulation"]);
  }
  catch (YAML::Exception& e)
  {
    TEMOTO_WARN("CONFIG: error parsing manipulation: %s", e.what());
  }
}

void RobotConfig::parseNavigation()
{
  try
  {
    feature_navigation_ = FeatureNavigation(yaml_config_["navigation"]);
  }
  catch (YAML::Exception e)
  {
    TEMOTO_ERROR("CONFIG: error parsing navigation: %s", e.what());
  }
}

std::string RobotConfig::toString() const
{
  std::string ret;
  ret += "ROBOT: " + getName() + "\n";
  ret += "  description : " + getDescription() + "\n";
  ret += "  reliability : " + std::to_string(getReliability()) + "\n";
  ret += "  features: \n";
  ret += feature_urdf_.isEnabled() ? "    urdf\n" : "";
  ret += feature_manipulation_.isEnabled() ? "    manipulation\n" : "";
  ret += feature_navigation_.isEnabled() ? "    navigation\n" : "";
  return ret;
}

}  // RobotManager namespace
