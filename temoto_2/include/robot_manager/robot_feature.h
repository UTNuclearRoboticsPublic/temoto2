#ifndef ROBOT_FEATURES_H
#define ROBOT_FEATURES_H

#include "common/temoto_id.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace robot_manager
{

class RobotFeature
{
public:
  RobotFeature(const std::string& name = "unknown_feature");

  std::string getName() const;

  std::string getPackageName() const;
  {
    return package_name_;
  }

  std::string getExecutable() const
  {
    return executable_;
  }

  temoto_id::ID getResourceId() const;
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

  void setResourceId(temoto_id::ID id)
  {
    resource_id_ = id;
  }

private:
  std::string name_;
  std::string package_name_;
  std::string executable_;
  temoto_id::ID resource_id_;
  bool feature_loaded_;
}

class FeatureURDF : public RobotFeature
{
  FeatureURDF();
  FeatureURDF(const YAML::Node& urdf_conf);
};

class FeatureManipulation : public RobotFeature
{
public:
  FeatureManipulation();
  FeatureManipulation(YAML::Node& manip_conf);

  std::vector<std::string> getPlanningGroups() const
  {
    return planning_groups_;
  }
  
  temoto_id::ID getDriverResourceId() const
  {
    return driver_resource_id_;
  }

  bool isDriverLoaded() const
  {
    return driver_loaded_;
  }

  bool setDriverLoaded(bool loaded)
  {
    feature_loaded_ = loaded;
  }

  void setDriverResourceId(temoto_id::ID id)
  {
    driver_resource_id_ = id;
  }

private:
  bool driver_loaded_;
  temoto_id::ID driver_resource_id_;
  std::vector<std::string> planning_groups_;
};


class FeatureNavigation : public RobotFeature
{
public:
  FeatureNavigation();
  FeatureNavigation(YAML::Node& nav_conf);

  const std::string& getGlobalPlanner() const
  {
    return global_planner_;
  }

  const std::string& getLocalPlanner() const
  {
    return local_planner_;
  }

  temoto_id::ID getDriverResourceId() const
  {
    return driver_resource_id_;
  }

  bool isDriverLoaded() const
  {
    return driver_loaded_;
  }

  bool setDriverLoaded(bool loaded)
  {
    driver_loaded_ = loaded;
  }

  void setDriverResourceId(temoto_id::ID id)
  {
    driver_resource_id_ = id;
  }

private:
  bool driver_loaded_;
  temoto_id::ID driver_resource_id_;
  std::string global_planner_;
  std::string local_planner_;
};

typedef std::vector<RobotFeature> Features;

//bool operator==(const Feature& feature1, const Feature& feature2);
}

#endif
