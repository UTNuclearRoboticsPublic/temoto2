#ifndef INTERFACE_ERROR_H
#define INTERFACE_ERROR_H
#include "temoto_error/temoto_error.h"
namespace interface_error
{
    enum interface_error : int
    {
        FORWARDING = 0     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        , NOT_INITIALIZED             // Failed to open rviz
    };
}

#endif
