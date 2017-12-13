#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "common/temoto_log_macros.h"
#include <yaml-cpp/yaml.h>
#include "common/reliability.h"

namespace robot_manager
{

class RobotConfig
{
public:
  /**
   * @brief RobotConfig
   */

  RobotConfig(YAML::Node yaml_config);
  


  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this robot is defined
  std::string getTemotoNamespace() const
  {
    return temoto_namespace_;
  }

  // Get the robot's namespace
  std::string getRobotNamespace() const
  {
    return temoto_namespace_ + '/' + robot_name_;
  }

  /// Get name
  std::string getName() const;

  // Get robot package name
  std::string getPackageName() const;

  // Get executable
  std::string getExecutable() const;

  // Get description
  std::string getDescription() const;

  std::string getUrdfPath() const;

  std::string getMoveitPackage() const;

  std::vector<std::string> getMoveitPlanningGroups() const;

  float getReliability() const
  {
    return reliability_.getReliability();
  };

  void adjustReliability(float reliability)
  {
    reliability_.adjustReliability(reliability);
  }

  void resetReliability(float reliability)
  {
    reliability_.resetReliability(reliability);
  }

  YAML::Node getYAMLConfig() const
  {
    return yaml_config_;
  }


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */
  void setTemotoNamespace(std::string temoto_namespace)
  {
    temoto_namespace_ = temoto_namespace;
  }

  void setName(std::string name)
  {
    robot_name_ = name;
  }

  void setPackageName(std::string package_name)
  {
    package_name_ = package_name;
  }

  void setExecutable(std::string executable)
  {
    executable_ = executable;
  }

  void setDescription(std::string description)
  {
    description_ = description;
  }


private:

  std::string log_class_ = "RobotConfig";
  std::string log_subsys_ = "robot_manager";
  std::string log_group_ = "robot_manager";

  std::string temoto_namespace_;
  YAML::Node yaml_config_;
  
  std::string robot_name_;
  std::string package_name_;
  std::string executable_;
  std::string description_;
  Reliability reliability_;
};

typedef std::shared_ptr<RobotConfig> RobotConfigPtr;
typedef std::vector<RobotConfigPtr> RobotConfigs;

static bool operator==(const RobotConfig& r1, const RobotConfig& r2)
{
  return (r1.getTemotoNamespace() == r2.getTemotoNamespace() && r1.getName() == r2.getName() &&
          r1.getExecutable() == r2.getExecutable() && r1.getPackageName() == r2.getPackageName());
}

} // robot_manager namespace

//namespace YAML
//{
//template <>
//struct convert<robot_manager::RobotConfig>
//{
//  static Node encode(const robot_manager::RobotConfig& config)
//  {
//    Node node;
//    node["robot_name"] = config.getName();
//    node["package_name"] = config.getPackageName();
//    node["executable"] = config.getExecutable();
//    node["description"] = config.getDescription();
//    node["reliability"] = config.getReliability();
//    return node;
//  }
//
//  static bool decode(const Node& node, robot_manager::RobotConfig& config)
//  {
//    if (!node.IsMap() || node.size() < 3)
//    {
//      return false;
//    }
//
//    try
//    {
//      config.setName(node["robot_name"].as<std::string>());
//      config.setPackageName(node["package_name"].as<std::string>());
//      config.setExecutable(node["executable"].as<std::string>());
//    }
//    catch (YAML::InvalidNode e)
//    {
//      return false;
//    }
//
//    try
//    {
//      config.setDescription(node["description"].as<std::string>());
//    }
//    catch (YAML::InvalidNode e)
//    {
//    }
//
//    try
//    {
//      config.resetReliability(node["reliability"].as<float>());
//    }
//    catch (YAML::InvalidNode e)
//    {
//    }
//
//    return true;
//  }
//};
//
//}
#endif
