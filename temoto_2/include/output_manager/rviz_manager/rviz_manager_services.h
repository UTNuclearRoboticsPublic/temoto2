#ifndef RVIZ_MANAGER_SERVICES_H
#define RVIZ_MANAGER_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"
#include "temoto_2/LoadRvizPlugin.h"

namespace rviz_manager
{
	namespace srv_name
	{
		const std::string MANAGER = "/temoto_2/rviz_manager";
		const std::string SERVER = "load_rviz_plugin";
	}
}

static bool operator==(const temoto_2::LoadRvizPlugin::Request& r1,
		const temoto_2::LoadRvizPlugin::Request& r2)
{
	return(
			r1.type == r2.type &&
			r1.name == r2.name &&
			r1.topic == r2.topic &&
			r1.config == r2.config
		  );
}
#endif