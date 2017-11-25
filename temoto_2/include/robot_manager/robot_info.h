#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "common/temoto_log_macros.h"
#include <yaml-cpp/yaml.h>

namespace robot_manager
{

class RobotInfo
{
public:
  /**
   * @brief RobotInfo
   */

  RobotInfo(std::string robot_name = "A noname robot");
  
  /**
   * \brief Adjust reliability
   * \param reliability the new reliability contribution to the moving average filter. The value has
   * to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void adjustReliability(float reliability = 1.0);


  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this robot is defined
  std::string getTemotoNamespace() const
  {
    return temoto_namespace_;
  }

  /// Get name
  std::string getName() const
  {
    return robot_name_;
  }

  // Get robot package name
  std::string getPackageName() const
  {
    return package_name_;
  }

  // Get executable
  std::string getExecutable() const
  {
    return executable_;
  }

  // Get description
  std::string getDescription() const
  {
    return description_;
  }

  float getReliability() const
  {
    return reliability_;
  };


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

  void setTopic(std::string topic)
  {
    topic_ = topic;
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

  /**
   * \brief Set reliability
   * \param reliability Sets the initial values for the reliability moving average filter.
   * The value has to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void setReliability(float reliability = 0.8);

private:

  std::string log_class_ = "RobotInfo";
  std::string log_subsys_ = "robot_manager";
  std::string log_group_ = "robot_manager";

  std::string temoto_namespace_;
  std::string robot_name_;
  std::string topic_;
  std::string package_name_;
  std::string executable_;
  std::string description_;

  /**
   * @brief Reliability ratings of the robot.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Average reliability.
   */
  float reliability_;

  /**
   * @brief Reliability rating of the robot.
   */
  unsigned int reliability_idx_;
};

typedef std::shared_ptr<RobotInfo> RobotInfoPtr;
typedef std::vector<RobotInfoPtr> RobotInfos;

static bool operator==(const RobotInfo& r1, const RobotInfo& r2)
{
  return (r1.getTemotoNamespace() == r2.getTemotoNamespace() && r1.getName() == r2.getName() &&
          r1.getExecutable() == r2.getExecutable() && r1.getPackageName() == r2.getPackageName());
}

} // robot_manager namespace

namespace YAML
{
template <>
struct convert<robot_manager::RobotInfo>
{
  static Node encode(const robot_manager::RobotInfo& robot_info)
  {
    Node node;
    node["robot_name"] = robot_info.getName();
    node["package_name"] = robot_info.getPackageName();
    node["executable"] = robot_info.getExecutable();
    node["description"] = robot_info.getDescription();
    node["reliability"] = robot_info.getReliability();
    return node;
  }

  static bool decode(const Node& node, robot_manager::RobotInfo& robot_info)
  {
    if (!node.IsMap() || node.size() < 3)
    {
      return false;
    }

    try
    {
      robot_info.setName(node["robot_name"].as<std::string>());
      robot_info.setPackageName(node["package_name"].as<std::string>());
      robot_info.setExecutable(node["executable"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
      return false;
    }

    try
    {
      robot_info.setDescription(node["description"].as<std::string>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    try
    {
      robot_info.setReliability(node["reliability"].as<float>());
    }
    catch (YAML::InvalidNode e)
    {
    }

    return true;
  }
};

}
#endif
