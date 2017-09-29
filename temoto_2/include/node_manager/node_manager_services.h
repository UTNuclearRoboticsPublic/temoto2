#ifndef NODE_MANAGER_SERVICES_H
#define NODE_MANAGER_SERVICES_H

#include <string>
#include "temoto_2/LoadResource.h"
#include "temoto_2/UnloadResource.h"

#include "temoto_2/nodeSpawnKill.h"
#include "temoto_2/resourceStatus.h"

namespace node_manager
{
	namespace srv_name
	{
		const std::string SPAWN_KILL = "/temoto_2/node_manager/spawn_kill";
	}
}


	// Define the equality operators for load/unload service requests.
	// This will define whether the resource server fires our callback
	// or considers resource already open and returns prevous response.
	static bool operator==(const temoto_2::LoadResource::Request& r1,
			const temoto_2::LoadResource::Request& r2)
	{
		return(
				r1.action == r2.action &&
				r1.package == r2.package &&
				r1.name == r2.name
			  );
	}

#endif
