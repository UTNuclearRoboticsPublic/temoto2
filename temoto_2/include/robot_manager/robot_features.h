#ifndef ROBOT_FEATURES_H
#define ROBOT_FEATURES_H

#include "common/temoto_id.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace robot_manager
{

  // Base class for all features
class RobotFeature
{
public:
  RobotFeature(const std::string& name = "unknown_feature");

  std::string getName() const;

  std::string getPackageName() const
  {
    return package_name_;
  }

  std::string getExecutable() const
  {
    return executable_;
  }

  std::string getArgs() const
  {
    return args_;
  }

  temoto_id::ID getResourceId() const
  {
    return resource_id_;
  }
  
  bool isLoaded() const 
  {
    return feature_loaded_;
  }

  bool setLoaded(bool loaded)
  {
    feature_loaded_ = loaded;
  }

  bool isEnabled() const 
  {
    return feature_enabled_;
  }

  bool setEnabled(bool enabled)
  {
    feature_enabled_ = enabled;
  }

  void setResourceId(temoto_id::ID id)
  {
    resource_id_ = id;
  }

protected:
  std::string name_;
  std::string package_name_;
  std::string executable_;
  std::string args_;
  temoto_id::ID resource_id_;
  bool feature_enabled_;
  bool feature_loaded_;
};

  // Base class for features which contain a robot driver
class FeatureWithDriver : public RobotFeature
{
public:
  FeatureWithDriver(const std::string& name = "unknown_feature_with_driver");

  bool isDriverLoaded() const
  {
    return driver_loaded_;
  }

  bool setDriverLoaded(bool loaded)
  {
    driver_loaded_ = loaded;
  }

  bool isDriverEnabled() const
  {
    return driver_enabled_;
  }

  bool setDriverEnabled(bool enabled)
  {
    driver_enabled_ = enabled;
  }

  temoto_id::ID getDriverResourceId() const
  {
    return driver_resource_id_;
  }

  void setDriverResourceId(temoto_id::ID id)
  {
    driver_resource_id_ = id;
  }

  std::string getDriverPackageName() const
  {
    return driver_package_name_;
  }

  std::string getDriverExecutable() const
  {
    return driver_executable_;
  }

  std::string getDriverArgs() const
  {
    return driver_args_;
  }

protected:
  bool driver_loaded_;
  bool driver_enabled_;
  std::string driver_package_name_;
  std::string driver_executable_;
  std::string driver_args_;
  temoto_id::ID driver_resource_id_;
};

// URDF feature
class FeatureURDF : public RobotFeature
{
  public:
  FeatureURDF();
  FeatureURDF(const YAML::Node& urdf_conf);
};


// Manipulation feature
class FeatureManipulation : public FeatureWithDriver
{
public:
  FeatureManipulation();
  FeatureManipulation(const YAML::Node& manip_conf);

  std::vector<std::string> getPlanningGroups() const
  {
    return planning_groups_;
  }

  std::string getActivePlanningGroup() const
  {
    return active_planning_group_;
  }

  std::string setActivePlanningGroup(std::string planning_group_name)
  {
    //TODO: check if group exists
    active_planning_group_ = planning_group_name;
  }
  
private:
  std::vector<std::string> planning_groups_;
  std::string active_planning_group_;
};


// Navigation feature
class FeatureNavigation : public FeatureWithDriver
{
public:
  FeatureNavigation();
  FeatureNavigation(const YAML::Node& nav_conf);

  const std::string& getGlobalPlanner() const
  {
    return global_planner_;
  }

  const std::string& getLocalPlanner() const
  {
    return local_planner_;
  }

private:
  std::string global_planner_;
  std::string local_planner_;
};

//bool operator==(const Feature& feature1, const Feature& feature2);
}

#endif
