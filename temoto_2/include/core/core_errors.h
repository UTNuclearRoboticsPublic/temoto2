#include "base_error/base_error.h"

namespace coreErr
{
    enum coreError : int
    {
        FORWARDING,         // Forwarding the error

        DESC_OPEN_FAIL,     // Failed to open the xml file
        DESC_NO_ROOT,       // Missing root element
        DESC_NO_ATTR,       // Attribute missing
        DESC_INVALID_ARG    // Invalid/Corrupt arguments
    };
}
