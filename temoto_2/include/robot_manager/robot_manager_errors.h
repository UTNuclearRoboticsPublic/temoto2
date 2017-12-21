#ifndef ROBOT_MANAGER_ERRORS_H
#define ROBOT_MANAGER_ERRORS_H

#include "base_error/base_error.h"

namespace robot_manager
{
enum ErrorCode : int
{
  // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero
  FORWARDING = 0,
  SERVICE_REQ_FAIL,      // Service request failed
  SERVICE_STATUS_FAIL,   // Service responded with FAILED status
  RESOURCE_UNLOAD_FAIL,  // Resource unload failed
  NULL_PTR,              // Pointer is null
  NOT_INITIALIZED        // Interface is not initialized
};
}

#endif
