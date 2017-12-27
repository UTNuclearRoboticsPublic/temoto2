#ifndef SENSOR_MANAGER_SERVICES_H
#define SENSOR_MANAGER_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"
#include "temoto_2/LoadAlgorithm.h"

namespace algorithm_manager
{
    // TODO: Change the srv_name to something more reasonable
    namespace srv_name
    {
        const std::string MANAGER = "algorithm_manager";
        const std::string SERVER = "load_algorithm";
        const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync";
    }
}

static bool operator==(const temoto_2::LoadAlgorithm::Request& r1,
    const temoto_2::LoadAlgorithm::Request& r2)
{
    return( r1.algorithm_type == r2.algorithm_type &&
            r1.package_name == r2.package_name &&
            r1.executable == r2.executable
    );
}
#endif