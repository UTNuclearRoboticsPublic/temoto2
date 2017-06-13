#include "core/task_handler/task_info.h"

std::string TaskInfo::getPath()
{
    return this->path_;
}

std::string TaskInfo::getName()
{
    return this->name_;
}

ParamList TaskInfo::getArgs()
{
    return this->args_;
}

ParamList TaskInfo::getReturn()
{
    return this->return_;
}
