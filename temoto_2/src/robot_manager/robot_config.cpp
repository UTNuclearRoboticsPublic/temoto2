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
    parseHardware();
  }
  catch (...)
  {
    TEMOTO_ERROR("Unable to create robot config: robot_name and hardware section with package_name "
                 "and executable fields are requred.");
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
  parseGripper();
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
    TEMOTO_WARN("CONFIG: description NOT FOUND");
  }
}

void RobotConfig::parseHardware() 
{
  try
  {
    std::string package_name = yaml_config_["hardware"]["package_name"].as<std::string>();
    std::string executable = yaml_config_["hardware"]["executable"].as<std::string>();
    features_.emplace_back(FeatureType::HARDWARE, package_name, executable);
  }
  catch (...)
  {
    TEMOTO_ERROR("CONFIG: hardware: (executable or package_path) NOT FOUND");
    //\TODO: throw
  }
}

void RobotConfig::parseUrdf()
{
  try
  {
    std::string package_name = yaml_config_["urdf"]["package_name"].as<std::string>();
    std::string executable = yaml_config_["urdf"]["executable"].as<std::string>();

    // Ingore pkg_name when executable is defined using absolute path (starts with /)
    //if (urdf_path.size() && urdf_path.front() != '/')
    //{
    //  std::string urdf_pkg = yaml_config_["urdf"]["package_name"].as<std::string>();
    //  urdf_path = urdf_pkg + '/' + urdf_path;
    //}
    features_.emplace_back(FeatureType::URDF, package_name, executable);
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
    std::string package_name = yaml_config_["manipulation"]["moveit_config_package"].as<std::string>();
    std::vector<std::string> groups;
    try
    {
      YAML::Node yaml_groups = yaml_config_["manipulation"]["planning_groups"];
      for (YAML::const_iterator it = yaml_groups.begin(); it != yaml_groups.end(); ++it)
      {
        groups.emplace_back(it->as<std::string>());
      }
    }
    catch (YAML::Exception& e)
    {
      TEMOTO_ERROR("CONFIG: manipulation: planning_groups NOT FOUND: %s", e.what());
    }
    
    planning_groups_ = groups;
    features_.emplace_back(FeatureType::MANIPULATION, package_name, "move_group.launch");
  }
  catch (YAML::Exception& e)
  {
    //TEMOTO_WARN("CONFIG: manipulation: moveit_package NOT FOUND: %s", e.what());
  }
}

void RobotConfig::parseNavigation()
{
  try
  {
    std::string package_name = yaml_config_["navigation"]["package_name"].as<std::string>();
    std::string executable = yaml_config_["navigation"]["executable"].as<std::string>();
    features_.emplace_back(FeatureType::NAVIGATION, package_name, executable);
  }
  catch (YAML::Exception e)
  {
    //TEMOTO_ERROR("CONFIG: navigation: (executable or package_path) NOT FOUND");
    //\TODO: throw
  }
}

void RobotConfig::parseGripper()
{
  //\TODO: IMPLEMENT GRIPPER
}

RobotFeature& RobotConfig::getRobotFeature(FeatureType type)
{
  auto it = std::find_if(features_.begin(), features_.end(),
                    [&](RobotFeature& f) -> bool { return f.getType() == type; });
  if (it == features_.end())
  {
    throw "TODO: EXCEPTION FEATURE NOT FOUND";
  }
  return *it;
}

std::string RobotConfig::toString() const
{
  std::string ret;
  ret += "ROBOT: " + getName() + "\n";
  ret += "  description : " + getDescription() + "\n";
  ret += "  reliability : " + std::to_string(getReliability()) + "\n";
  ret += "  features: \n";
  for (auto& f : features_)
  {
    ret += "    " + f.getName() + "\n";
  }
  return ret;
}

}  // RobotManager namespace
