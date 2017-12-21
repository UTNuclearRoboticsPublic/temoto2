#ifndef CONTEXT_MANAGER_ERRORS_H
#define CONTEXT_MANAGER_ERRORS_H

#include "temoto_error/temoto_error.h"

namespace context_manager
{
enum ErrorCode : int
{
  FORWARDING = error::FORWARDING_CODE,  // Code 0 errors always indicate the forwarding type, hence
                                        // it has to be set manually to zero
  SERVICE_REQ_FAIL,                     // Service request failed
  RESOURCE_LOAD_FAIL,                   // Resource unload failed
  NOT_INITIALIZED                       // Interface is not initialized
};
}

#endif
