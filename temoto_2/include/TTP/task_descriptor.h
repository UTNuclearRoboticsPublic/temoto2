#ifndef TASK_DESCRIPTOR_H
#define TASK_DESCRIPTOR_H

#include "TTP/io_descriptor.h"
//#include "TTP/base_task/base_task.h"

namespace TTP
{

typedef std::string Action;

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

    TaskDescriptor(){}

    TaskDescriptor( Action action );

    TaskDescriptor( Action action, IODescriptor& input_descriptor );

    TaskDescriptor( Action action, TaskInterface& task_interface );

    TaskDescriptor( Action action, std::vector<TaskInterface>& task_interfaces );

    friend std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td);

    const Action& getAction() const;

    std::vector<TaskInterface>& getInterfaces();

    const IODescriptor& getFirstInputDescriptor();

    bool empty();

private:
    std::vector<TaskInterface> task_interfaces_;

    Action action_;

    std::string task_class_name_;

    std::string task_package_name_;

    std::string task_lib_path_;

    //boost::shared_ptr<Task> task_pointer_;
};
}

#endif
