#ifndef ROBOT_MANAGER_SERVICES_H
#define ROBOT_MANAGER_SERVICES_H

#include <string>
#include "temoto_2/RobotLoad.h"
#include "temoto_2/RobotPlan.h"
#include "temoto_2/RobotExecute.h"
#include "temoto_2/RobotSetTarget.h"
#include "temoto_2/RobotSetMode.h"
#include "temoto_2/RobotGetVizInfo.h"

namespace robot_manager
{
namespace srv_name
{
const std::string MANAGER = "robot_manager";
const std::string SYNC_TOPIC = "/temoto_2/" + MANAGER + "/sync";

const std::string SERVER_LOAD = "load";
const std::string SERVER_PLAN = "plan";
const std::string SERVER_EXECUTE = "execute";
const std::string SERVER_GET_VIZ_INFO = "get_visualization_info";
const std::string SERVER_SET_TARGET = "set_target";
const std::string SERVER_SET_MODE = "set_mode";
}

namespace modes
{
const std::string AUTO = "auto";
const std::string NAVIGATION = "navigation";
const std::string MANIPULATION = "manipulation";
}
}

static bool operator==(const temoto_2::RobotLoad::Request& r1,
                       const temoto_2::RobotLoad::Request& r2)
{
  return (r1.robot_name == r2.robot_name);
}
#endif
