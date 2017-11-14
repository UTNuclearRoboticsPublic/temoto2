#ifndef SENSOR_MANAGER_SERVICES_H
#define SENSOR_MANAGER_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"
#include "temoto_2/ListDevices.h"
#include "temoto_2/LoadSensor.h"

namespace sensor_manager
{
	namespace srv_name
	{
		const std::string MANAGER = "sensor_manager";
		const std::string SERVER = "start_sensor";
		const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync";
	}
}

static bool operator==(const temoto_2::LoadSensor::Request& r1,
		const temoto_2::LoadSensor::Request& r2)
{
	return(
			r1.sensor_type == r2.sensor_type &&
			r1.package_name == r2.package_name &&
			r1.executable == r2.executable
		  );
}
#endif
