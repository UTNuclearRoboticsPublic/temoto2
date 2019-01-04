#ifndef PROCESS_MANAGER_SERVICES_H
#define PROCESS_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_2/LoadProcess.h"

namespace process_manager
{
	namespace srv_name
	{
		const std::string MANAGER = "process_manager";
		const std::string SERVER = "load_process";
	}

  namespace action
  {
		const std::string ROS_EXECUTE = "ros_execute";
		const std::string SYS_EXECUTE = "sys_execute";
  }
}


// Define the equality operators for load/unload service requests.
// This will define whether the resource server fires our callback
// or considers resource already open and returns prevous response.
static bool operator==(const temoto_2::LoadProcess::Request& r1,
		const temoto_2::LoadProcess::Request& r2)
{
	return(
			r1.action == r2.action &&
			r1.package_name == r2.package_name &&
			r1.executable == r2.executable &&
			r1.args == r2.args
		  );
}

#endif
