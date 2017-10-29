#ifndef HUMAN_CONTEXT_SERVICES_H
#define HUMAN_CONTEXT_SERVICES_H

#include <string>
#include "temoto_2/RobotLoad.h"
#include "temoto_2/RobotPlan.h"
#include "temoto_2/RobotExecute.h"
#include "temoto_2/RobotGetRvizConfig.h"

namespace robot_manager
{
	namespace srv_name
	{
		const std::string MANAGER = "robot_manager";

		const std::string SERVER_LOAD = "/temoto_2/robot_manager/load";
		const std::string SERVER_PLAN = "/temoto_2/robot_manager/plan";
		const std::string SERVER_EXECUTE = "/temoto_2/robot_manager/execute";
		const std::string SERVER_GET_RVIZ_CONFIG = "/temoto_2/robot_manager/get_rviz_config";
		const std::string SERVER_SET_TARGET = "/temoto_2/robot_manager/set_target";
	}
}

static bool operator==(const temoto_2::RobotLoad::Request& r1,
		const temoto_2::RobotLoad::Request& r2)
{
  return (r1.robot_name == r2.robot_name);
}
#endif
