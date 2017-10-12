#include "TTP/task_descriptor.h"

namespace TTP
{

TaskDescriptor::TaskDescriptor( IODescriptor& input_descriptor )
{
    // Create an interface
    TaskInterface task_interface;
    task_interface.input_descriptor = input_descriptor;

    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( std::move( task_interface ) );
}

TaskDescriptor::TaskDescriptor( TaskInterface& task_interface )
{
    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( task_interface );
}

TaskDescriptor::TaskDescriptor( std::vector<TaskInterface>& task_interfaces ) : task_interfaces_( task_interfaces ){}

}
