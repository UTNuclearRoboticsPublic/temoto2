#ifndef CORE_ERRORS_H
#define CORE_ERRORS_H

#include "base_error/base_error.h"

namespace coreErr
{
    enum coreError : int
    {
        FORWARDING = error::FORWARDING_CODE,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        DESC_OPEN_FAIL,     // Failed to open the xml file
        DESC_NO_ROOT,       // Missing root element
        DESC_NO_ATTR,       // Attribute missing
        DESC_INVALID_ARG,   // Invalid/Corrupt arguments
        CLASS_LOADER_FAIL,  // Classloader failed to do its job
        FIND_TASK_FAIL,     // Failed to find tasks
        UNHANDLED,          // Unhandled exception
        UNSPECIFIED_TASK,   // The task is unspecified
        NAMELESS_TASK_CLASS,
        NO_TASK_CLASS       // Task handler could not find the task class
    };
}

#endif
