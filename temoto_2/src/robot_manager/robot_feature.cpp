#include "robot_manager/robot_feature.h"

namespace robot_manager
{
RobotFeature::RobotFeature(const FeatureType type) : RobotFeature(type, "", "")
{
}

RobotFeature::RobotFeature(const FeatureType type, const std::string package_name, const std::string executable,
                           std::string args)
  : type_(type), package_name_(package_name), executable_(executable), args_(args)
{
}

std::string RobotFeature::getName() const
{
  switch (type_)
  {
    case FeatureType::URDF:
      return "urdf";
    case FeatureType::MANIPULATION:
      return "manipulation";
    case FeatureType::NAVIGATION:
      return "navigation";
    case FeatureType::GRIPPER:
      return "gripper";
    default:
      return "unknown_feature_type";
  }
}

void RobotFeature::setResourceId(temoto_id::ID resource_id)
{
  resource_id_ = resource_id;
}

bool operator==(const RobotFeature& rf1, const RobotFeature& rf2)
{
  return (rf1.getType() == rf2.getType() && rf1.getPackageName() == rf2.getPackageName() &&
          rf1.getExecutable() == rf2.getExecutable() && rf1.getArgs() == rf2.getArgs());
}

}
