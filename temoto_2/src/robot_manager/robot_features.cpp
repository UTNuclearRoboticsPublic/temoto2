#include "robot_manager/robot_features.h"
#include <iostream>

// ALL OF THESE CLASSES MAY THROW YAML EXCEPTIONS

namespace robot_manager
{
RobotFeature::RobotFeature(const std::string& name) : name_(name), feature_loaded_(false), feature_enabled_(false)
{
}

FeatureWithDriver::FeatureWithDriver(const std::string& name)
  : RobotFeature(name), driver_loaded_(false), driver_enabled_(false)
{
}

FeatureURDF::FeatureURDF() : RobotFeature("urdf")
{
}

FeatureURDF::FeatureURDF(const YAML::Node& urdf_conf) : RobotFeature("urdf")
{
  this->package_name_ = urdf_conf["package_name"].as<std::string>();
  this->executable_ = urdf_conf["executable"].as<std::string>();
  this->feature_enabled_ = true;
}

FeatureManipulation::FeatureManipulation() : FeatureWithDriver("manipulation")
{
}

FeatureManipulation::FeatureManipulation(const YAML::Node& manip_conf)
  : FeatureWithDriver("manipulation")
{
  this->package_name_ = manip_conf["controller"]["package_name"].as<std::string>();
  if (manip_conf["controller"]["executable"].IsNull())
  {
    this->executable_ = "move_group.launch";
  }
  else
  {
    this->executable_ = manip_conf["controller"]["executable"].as<std::string>();
  }

  // parse planning groups
  YAML::Node yaml_groups = manip_conf["controller"]["planning_groups"];
  for (YAML::const_iterator it = yaml_groups.begin(); it != yaml_groups.end(); ++it)
  {
    planning_groups_.emplace_back(it->as<std::string>());
  }
  this->feature_enabled_ = true;

  this->driver_package_name_ = manip_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable_ = manip_conf["driver"]["executable"].as<std::string>();
  this->driver_enabled_ = true;
}

FeatureNavigation::FeatureNavigation() : FeatureWithDriver("navigation")
{
}

FeatureNavigation::FeatureNavigation(const YAML::Node& nav_conf) : FeatureWithDriver("navigation")
{
  this->package_name_ = nav_conf["controller"]["package_name"].as<std::string>();
  this->executable_ = "move_base.launch";
  this->global_planner_ = nav_conf["controller"]["global_planner"].as<std::string>();
  this->local_planner_ = nav_conf["controller"]["local_planner"].as<std::string>();
  this->feature_enabled_ = true;

  this->driver_package_name_ = nav_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable_ = nav_conf["driver"]["executable"].as<std::string>();
  this->driver_enabled_ = true;
}

// bool operator==(const RobotFeature& rf1, const RobotFeature& rf2)
//{
//  return (rf1.getType() == rf2.getType() && rf1.getPackageName() == rf2.getPackageName() &&
//          rf1.getExecutable() == rf2.getExecutable() && rf1.getArgs() == rf2.getArgs());
//}
}
