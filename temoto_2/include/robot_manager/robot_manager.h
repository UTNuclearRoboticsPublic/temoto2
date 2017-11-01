#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "core/common.h"
#include "common/temoto_id.h"
#include "robot_manager/robot_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include "package_info/package_info.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <map>
#include "robot_manager/robot.h"

#include "leap_motion_controller/Set.h"

#include <mutex>


namespace robot_manager
{
class RobotManager
{
public:
  RobotManager();

  /**
   * @brief Service that loads a robot
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  bool loadCb(temoto_2::RobotLoad::Request& req, temoto_2::RobotLoad::Response& res);

  /**
   * @brief Service that plans using moveit
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
    bool setTargetCb(temoto_2::RobotSetTarget::Request& req, temoto_2::RobotSetTarget::Response&
    res);
  
  const std::string& getName() const
  {
    return log_subsys_;
  }

  std::vector<PackageInfoPtr> pkg_infos_;

private:
  PackageInfoPtr findRobot(temoto_2::LoadProcess::Request& req, const std::string& robot_name);

  bool getRvizConfigCb(temoto_2::RobotGetRvizConfig::Request& req,
                       temoto_2::RobotGetRvizConfig::Response& res);

  void targetPoseCb(const leap_motion_controller::Set& set);

  // Currently one robot at the time, add vec or map in future.
  std::shared_ptr<Robot> active_robot_;
  geometry_msgs::PoseStamped default_target_pose_;

  std::string log_class_, log_subsys_, log_group_;

  ros::NodeHandle nh_;
  ros::ServiceServer server_load_;
  ros::ServiceServer server_plan_;
  ros::ServiceServer server_exec_;
  ros::ServiceServer server_get_rviz_cfg_;
  ros::ServiceServer server_set_target_;

  ros::Subscriber target_pose_sub_;

  // store active
  temoto_id::ID active_robot_id_;
  temoto_2::RobotLoad active_load_msg_;

  // Resource manager for contacting process manager
  rmp::ResourceManager<RobotManager> resource_manager_;
  std::mutex default_pose_mutex_;
};
}

#endif
