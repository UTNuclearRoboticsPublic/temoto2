#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr
#include "temoto_2/RobotInfoSync.h"
#include "common/temoto_log_macros.h"

namespace robot_manager
{
namespace sync_action
{
//const std::string ADD = "add";
const std::string UPDATE = "update";
const std::string GET_ROBOTS = "get_robots";
}

class RobotInfo
{
public:
  /**
   * @brief RobotInfo
   */

  RobotInfo(std::string robot_name = "A noname robot");
  
  RobotInfo(const temoto_2::RobotInfoSync& msg);

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
    return msg_.temoto_namespace;
  }

  /// Get name
  std::string getName() const
  {
    return msg_.robot_name;
  }


  // Get robot type
  std::string getType() const
  {
    return msg_.robot_type;
  }

  // Get robot package name
  std::string getPackageName() const
  {
    return msg_.package_name;
  }

  // Get executable
  std::string getExecutable() const
  {
    return msg_.executable;
  }

  // Get description
  std::string getDescription() const
  {
    return msg_.description;
  }

  float getReliability() const
  {
    return msg_.reliability;
  };

  // get sync message with proper sync_action (ADD or UPDATE)
  const temoto_2::RobotInfoSync& getSyncMsg(const std::string& action);


  /* * * * * * * * * * * *
   *     SETTERS
   * * * * * * * * * * * */
  void setTemotoNamespace(std::string temoto_namespace)
  {
    msg_.temoto_namespace = temoto_namespace;
  }

  void setName(std::string name)
  {
    msg_.robot_name = name;
  }

  void setTopic(std::string topic)
  {
    msg_.topic = topic;
  }

  void setType(std::string robot_type)
  {
    msg_.robot_type = robot_type;
  }

  void setPackageName(std::string package_name)
  {
    msg_.package_name = package_name;
  }

  void setExecutable(std::string executable)
  {
    msg_.executable = executable;
  }

  void setDescription(std::string description)
  {
    msg_.description = description;
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
  
  /**
   * @brief Message container where the robot info is stored.
   */
  temoto_2::RobotInfoSync msg_;

  /**
   * @brief Reliability ratings of the robot.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Reliability moving average.
   */
  float reliability_average;

  /**
   * @brief Reliability rating of the robot.
   */
  unsigned int reliability_idx;
};

typedef std::shared_ptr<RobotInfo> RobotInfoPtr;

static bool operator==(const RobotInfo& s1, const RobotInfo& s2)
{
  return (s1.getTemotoNamespace() == s2.getTemotoNamespace() && s1.getTopic() == s2.getTopic() &&
          s1.getExecutable() == s2.getExecutable() && s1.getPackageName() == s2.getPackageName());
}
}
#endif
