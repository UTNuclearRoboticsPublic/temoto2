#include "base_error/base_error.h"

enum class coreErr : int
{
    FORWARDING,
    DESC_OPEN_FAIL,
    DESC_NO_ROOT,
    DESC_NO_ATTR,
    DESC_INVALID_ARG
};
