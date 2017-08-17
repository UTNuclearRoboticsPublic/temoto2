#include "base_error/base_error.h"

namespace outputManagerErr
{
    enum outputManagerError : int
    {
        FORWARDING = 0,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        RVIZ_OPEN_FAIL,     // Failed to open rviz
        SERVICE_REQ_FAIL,   // Failed to open rviz
        PLUGIN_LOAD_FAIL    // Failed to load rviz plugin

    };
}
