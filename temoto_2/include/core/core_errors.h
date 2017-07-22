#include "base_error/base_error.h"

namespace coreErr
{
    enum coreError : int
    {
        FORWARDING = 0,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        DESC_OPEN_FAIL,     // Failed to open the xml file
        DESC_NO_ROOT,       // Missing root element
        DESC_NO_ATTR,       // Attribute missing
        DESC_INVALID_ARG    // Invalid/Corrupt arguments
    };
}
