#ifndef TASK_ERRORS_H
#define TASK_ERRORS_H

#include "temoto_error/temoto_error.h"

namespace taskErr
{
    enum taskError : int
    {
        FORWARDING = 0,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        SERVICE_REQ_FAIL,   // Service request failed
        RESOURCE_UNLOAD_FAIL   // Resource unload failed
    };
}

#endif
