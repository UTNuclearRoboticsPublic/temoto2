#ifndef TASK_INFO_H
#define TASK_INFO_H

#include "core/common.h"

class TaskInfo
{
    friend class DescriptionProcessor;

public:

    std::string getPath();

    std::string getName();

    ParamList getArgs();

    ParamList getReturn();

private:

    std::string name_;

    std::string path_;

    ParamList args_;

    ParamList return_;
};

#endif
