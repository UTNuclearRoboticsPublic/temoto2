#ifndef RUNNING_TASK_H
#define RUNNING_TASK_H

#include "core/task_handler/task_info.h"
#include "base_task/task.h"

class RunningTask
{
public:

    /**
     * @brief task_info_
     */
    TaskInfo task_info_;

    /**
     * @brief running_thread_
     */
    std::thread task_thread_;

    /**
     * @brief task_pointer_
     */
    boost::shared_ptr<Task> task_pointer_;

private:

};

#endif
