#ifndef TASK_CONTAINER_H
#define TASK_CONTAINER_H

#include "TTP/base_task/task.h"
#include "TTP/task_descriptor.h"
#include <boost/shared_ptr.hpp>

namespace TTP
{

class TaskContainer
{
public:

    /**
     * @brief TaskContainer
     * @param pointer to the task
     * @param task_interface
     */
    TaskContainer(boost::shared_ptr<Task> task_pointer, boost::shared_ptr<TaskDescriptor> task_descriptor);

    Subjects operator()(Subjects input_subjects);

private:

    boost::shared_ptr<Task> task_pointer_;

    boost::shared_ptr<TaskDescriptor> task_descriptor_;
};
}// END of TTP namespace

#endif
