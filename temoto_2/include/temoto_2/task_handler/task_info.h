#ifndef TASK_INFO_H
#define TASK_INFO_H

#include "temoto_2/task_handler/common.h"
#include <string>
#include <vector>

class TaskInfo
{

public:

    TaskInfo();

    TaskInfo( std::string descPath );

    std::string getPath();

    std::string setPath();

    std::string getName();

    std::string setName();



private:

    std::string path_;

    paramList args_;

    paramList return_;
};

#endif
