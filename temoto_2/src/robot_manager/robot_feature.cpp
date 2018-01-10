#include "robot_manager/robot_feature.h"

// ALL OF THESE CLASSES MAY THROW YAML EXCEPTIONS

namespace robot_manager
{
FeatureURDF::FeatureURDF() : RobotFeature("urdf")
{
}

FeatureURDF::FeatureURDF(const YAML::Node& urdf_conf) : RobotFeature("urdf")
{
  this->package_name = urdf_conf["package_name"].as<std::string>();
  this->executable = urdf_conf["executable"].as<std::string>();
}

FeatureManipulation::FeatureManipulation(const YAML::Node& manip_conf)
  : RobotFeature("manipulation")
{
  this->package_name = manip_conf["controller"]["package_name"].as<std::string>();
  this->executable = "move_group.launch";
  
  this->driver_package_name = manip_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable = manip_conf["driver"]["executable"].as<std::string>();
}

FeatureNavigation::FeatureNavigation(const YAML::Node& manip_conf)
  : RobotFeature("navigation")
{
  this->package_name = manip_conf["controller"]["package_name"].as<std::string>();
  this->executable = "move_base.launch";
  
  this->driver_package_name = manip_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable = manip_conf["driver"]["executable"].as<std::string>();
}



// bool operator==(const RobotFeature& rf1, const RobotFeature& rf2)
//{
//  return (rf1.getType() == rf2.getType() && rf1.getPackageName() == rf2.getPackageName() &&
//          rf1.getExecutable() == rf2.getExecutable() && rf1.getArgs() == rf2.getArgs());
//}
}
