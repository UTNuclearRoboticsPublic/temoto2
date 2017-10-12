#ifndef TASK_DESCRIPTOR_H
#define TASK_DESCRIPTOR_H

#include "TTP/io_descriptor.h"
#include "TTP/base_task/base_task.h"

namespace TTP
{

struct TaskInterface
{
    IODescriptor input_descriptor;
    IODescriptor output_descriptor;
};

/**
 * @brief This class contains all the information needed for finding, loading
 * and executing a given task.
 */
class TaskDescriptor
{
public:

    TaskDescriptor( IODescriptor& input_descriptor );

    TaskDescriptor( TaskInterface& task_interface );

    TaskDescriptor( std::vector<TaskInterface>& task_interfaces );

private:
    std::vector<TaskInterface> task_interfaces_;

    std::string task_name_;

    std::string task_class_name_;

    std::string task_package_name_;

    std::string task_lib_path_;

    boost::shared_ptr<Task> task_pointer_;
};
}

#endif
