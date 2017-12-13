#ifndef ROBOT_FEATURE_H
#define ROBOT_FEATURE_H

#include "common/temoto_id.h"
#include <string>

namespace robot_manager
{
enum class FeatureType
{
  URDF,
  MANIPULATION,
  NAVIGATION,
  GRIPPER
};

class RobotFeature
{
public:
  RobotFeature(FeatureType type);
  FeatureType type_;
  std::string package_name_;
  std::string executable_;
  temoto_id::ID resource_id_;

  std::string getName() const;
  FeatureType getType() const
  {
    return type_;
  }

  bool operator<(const RobotFeature& feature) const;
};
}

#endif
