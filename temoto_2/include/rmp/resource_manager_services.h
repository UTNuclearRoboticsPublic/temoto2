#ifndef RESOURCE_MANAGER_SERVICES_H
#define RESOURCE_MANAGER_SERVICES_H

#include "temoto_2/UnloadResource.h"
#include "temoto_2/ResourceStatus.h"

namespace rmp
{
enum status_codes : int
{
  OK = 0,
  FAILED,
};

namespace srv_name
{
const std::string PREFIX = "/temoto_2";
}
}

#endif
