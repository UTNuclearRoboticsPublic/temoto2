#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/reliability.h"
#include "temoto_core/common/base_subsystem.h"
#include <yaml-cpp/yaml.h>
#include "robot_manager/robot_features.h"

namespace robot_manager
{

class RobotConfig : temoto_core::BaseSubsystem
{
public:
  /**
   * @brief RobotConfig
   */

  RobotConfig(YAML::Node yaml_config, temoto_core::BaseSubsystem& b);
  


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
  std::string getAbsRobotNamespace() const
  {
    return "/" + temoto_namespace_ + "/robot_manager/robots/" + name_;
  }

  void parseName();
  void parseHardware();

  void parseTemotoNamespace();
  void parseDescription();
  void parseReliability();

  void parseUrdf();
  void parseManipulation();
  void parseNavigation();

  std::string getName() const
  {
    return name_;
  }

  std::string getDescription() const
  {
    return description_;
  }

  float getReliability() const
  {
    return reliability_.getReliability();
  }

  FeatureURDF& getFeatureURDF()
  {
    return feature_urdf_;
  }

  FeatureManipulation& getFeatureManipulation()
  {
    return feature_manipulation_;
  }

  FeatureNavigation& getFeatureNavigation()
  {
    return feature_navigation_;
  }

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

  void setTemotoNamespace(std::string temoto_namespace)
  {
    temoto_namespace_ = temoto_namespace;
  }


private:

  std::string log_class_ = "RobotConfig";
  std::string log_subsys_ = "robot_manager";
  std::string log_group_ = "robot_manager";

  std::string temoto_namespace_;
  YAML::Node yaml_config_;

  FeatureURDF feature_urdf_;
  FeatureManipulation feature_manipulation_;
  FeatureNavigation feature_navigation_;
  
  std::string name_;
  std::string description_;
  temoto_core::Reliability reliability_;
};

typedef std::shared_ptr<RobotConfig> RobotConfigPtr;
typedef std::vector<RobotConfigPtr> RobotConfigs;

static bool operator==(const RobotConfig& r1, const RobotConfig& r2)
{
  //compare if the two configs are created from the same YAML config string
  return r1.getYAMLConfig() == r2.getYAMLConfig();

//  return (r1.getTemotoNamespace() == r2.getTemotoNamespace() && r1.getName() == r2.getName() &&
//          r1.getDescription() == r2.getDescription() &&
//          r1.getPlanningGroups() == r2.getPlanningGroups() &&
//          r1.getRobotFeatures() == r2.getRobotFeatures());
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
