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
    TaskContainer(boost::shared_ptr<Task> task_pointer, TaskInterface task_interface);

    std::vector<Subject> operator()(std::vector<Subject> input_subjects);

private:

    boost::shared_ptr<Task> task_pointer_;

    TaskInterface task_interface_;
};
}// END of TTP namespace

#endif
