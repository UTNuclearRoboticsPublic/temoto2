#ifndef TASK_INFO_H
#define TASK_INFO_H

#include "core/common.h"
#include <string>
#include <vector>

class TaskInfo
{
    friend class DescriptionProcessor;

public:

    std::string getPath();

    std::string getName();

    paramList getArgs();

    paramList getReturn();

private:

    std::string name_;

    std::string path_;

    paramList args_;

    paramList return_;
};

#endif
