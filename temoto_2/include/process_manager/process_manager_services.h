#ifndef PROCESS_MANAGER_SERVICES_H
#define PROCESS_MANAGER_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"

namespace process_manager
{
	namespace srv_name
	{
		const std::string MANAGER = "/temoto_2/process_manager";
		const std::string SERVER = "load_process";
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
				r1.package == r2.package &&
				r1.name == r2.name
			  );
	}

#endif
