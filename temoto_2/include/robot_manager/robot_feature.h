#ifndef ROBOT_FEATURE_H
#define ROBOT_FEATURE_H

#include "common/temoto_id.h"
#include <string>
#include <vector>

namespace robot_manager
{
enum class FeatureType
{
  HARDWARE,
  URDF,
  MANIPULATION,
  NAVIGATION,
  GRIPPER
};

class RobotFeature
{
public:
  RobotFeature(const FeatureType type);
  RobotFeature(const FeatureType type, const std::string package_name, const std::string executable, const std::string args = "");

  std::string getName() const;

  FeatureType getType() const
  {
    return type_;
  }

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

  bool isLoaded() const 
  {
    return is_loaded_;
  }

  void setResourceId(temoto_id::ID resource_id);
  void setLoaded(bool loaded)
  {
    is_loaded_ = loaded;
  }

private:
  FeatureType type_;
  std::string package_name_;
  std::string executable_;
  std::string args_;
  temoto_id::ID resource_id_;
  bool is_loaded_;

};

typedef std::vector<RobotFeature> RobotFeatures;

bool operator==(const RobotFeature& feature1, const RobotFeature& feature2);


}

#endif
