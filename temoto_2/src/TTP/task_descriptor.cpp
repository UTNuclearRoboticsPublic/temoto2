#include "TTP/task_descriptor.h"
#include <iostream>

namespace TTP
{

TaskDescriptor::TaskDescriptor( Action action ) : action_(action)
{
    // Create an interface
    TaskInterface task_interface;

    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( std::move( task_interface ) );
}

TaskDescriptor::TaskDescriptor( Action action, IODescriptor& input_descriptor ) : action_(action)
{
    // Create an interface
    TaskInterface task_interface;
    task_interface.input_descriptor = input_descriptor;

    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( std::move( task_interface ) );
}

TaskDescriptor::TaskDescriptor( Action action, TaskInterface& task_interface ) : action_(action)
{
    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( task_interface );
}

TaskDescriptor::TaskDescriptor( Action action, std::vector<TaskInterface>& task_interfaces )
    : action_(action),
      task_interfaces_( task_interfaces ){}

const IODescriptor& TaskDescriptor::getFirstInputDescriptor()
{
    return task_interfaces_[0].input_descriptor;
}

const Action& TaskDescriptor::getAction() const
{
    return action_;
}

std::vector<TaskInterface>& TaskDescriptor::getInterfaces()
{
    return task_interfaces_;
}

bool TaskDescriptor::empty()
{
    return task_interfaces_.empty();
}

std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td)
{
    stream << "Action: " << td.getAction() << std::endl;
    stream << "Interfaces:" << std::endl;
    for (auto& task_interface : td.task_interfaces_)
    {
        stream << task_interface.input_descriptor;
    }
    return stream;
}

}
