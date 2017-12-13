#include "robot_manager/robot_feature.h"

namespace robot_manager
{
RobotFeature::RobotFeature(FeatureType type) : type_(type)
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
      return "Unknown_Feature_Type";
  }
}

bool RobotFeature::operator<(const RobotFeature& feature) const
{
  return (getName() < feature.getName());
}
}
