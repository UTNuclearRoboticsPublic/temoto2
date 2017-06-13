#include "core/task_handler/task_info.h"

std::string getPath()
{
    return path_;
}

std::string getName()
{
    return name_;
}

paramList getArgs()
{
    return args_;
}

paramList getReturn()
{
    return return_;
}
