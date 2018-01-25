#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "robot_manager/robot_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "context_manager/context_manager_services.h"
#include "rmp/resource_manager.h"
#include "rmp/config_synchronizer.h"
#include "temoto_2/ConfigSync.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_manager/robot.h"
#include "robot_manager/robot_config.h"
#include "std_msgs/String.h"

#include "human_msgs/Hands.h"

#include <mutex>
#include <vector>
#include <map>

#include <tf/transform_listener.h>


namespace robot_manager
{
// Forward declaration
class Robot;

typedef std_msgs::String PayloadType;

class RobotManager : public BaseSubsystem
{
public:
  RobotManager();

  const std::string& getName() const
  {
    return log_subsys_;
  }

private:

  /**
   * @brief Callback for loading a robot
   * @param Request that specifies the robot's parameters
   * @param Returns which robot got loaded
   */
  void loadCb(temoto_2::RobotLoad::Request& req, temoto_2::RobotLoad::Response& res);

  /**
   * @brief Callback for unloading a robot
   * @param Request that specifies which robot to unload
   * @param Returns status information
   */
  void unloadCb(temoto_2::RobotLoad::Request& req, temoto_2::RobotLoad::Response& res);

  /**
   * @brief Service callback that plans using moveit
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  bool planCb(temoto_2::RobotPlan::Request& req, temoto_2::RobotPlan::Response& res);

  /**
   * @brief Service that executes the moveit plan
   * @param LoadGesture request message
   * @param LoadGesture response message
   * @return
   */

  bool execCb(temoto_2::RobotExecute::Request& req, temoto_2::RobotExecute::Response& res);

  /**
   * @brief Service that sets robot manager default target
   * @param LoadSpeech request message
   * @param LoadSpeech response message
   * @return
   */

  bool setTargetCb(temoto_2::RobotSetTarget::Request& req, temoto_2::RobotSetTarget::Response& res);

  bool setModeCb(temoto_2::RobotSetMode::Request& req, temoto_2::RobotSetMode::Response& res);

  void syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload);

  void advertiseConfig(RobotConfigPtr config);

  void advertiseConfigs(RobotConfigs configs);

  RobotConfigs parseRobotConfigs(const YAML::Node& config);

  RobotConfigPtr findRobot(const std::string& robot_name, const RobotConfigs& robot_infos);

  bool getVizInfoCb(temoto_2::RobotGetVizInfo::Request& req,
                       temoto_2::RobotGetVizInfo::Response& res);

  void targetPoseCb(const temoto_2::ObjectContainer& msg);

  void statusInfoCb(temoto_2::ResourceStatus& srv);

  void loadLocalRobot(RobotConfigPtr info_ptr, temoto_id::ID resource_id);


  typedef std::shared_ptr<Robot> RobotPtr;
  typedef std::map<temoto_id::ID, RobotPtr> Robots;
  RobotPtr active_robot_;
  Robots loaded_robots_;
  RobotConfigs local_configs_;
  RobotConfigs remote_configs_;

  std::string mode_;
  geometry_msgs::PoseStamped default_target_pose_;

  std::string log_class_, log_subsys_;

  ros::NodeHandle nh_;
  ros::ServiceServer server_plan_;
  ros::ServiceServer server_exec_;
  ros::ServiceServer server_get_viz_cfg_;
  ros::ServiceServer server_set_target_;
  ros::ServiceServer server_set_mode_;

  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_get_viz_cfg_;
  ros::ServiceClient client_set_target_;
  ros::ServiceClient client_set_mode_;

  ros::Subscriber target_pose_sub_;
  temoto_2::LoadGesture hand_srv_msg_;
  
  // Keeps robot_infos in sync with other managers
  rmp::ConfigSynchronizer<RobotManager, PayloadType> config_syncer_;

  // Resource manager for contacting process manager
  rmp::ResourceManager<RobotManager> resource_manager_;
  std::mutex default_pose_mutex_;


  tf::TransformListener tf_listener;

};
}

#endif
