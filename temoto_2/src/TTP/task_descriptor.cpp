#include "TTP/task_descriptor.h"
#include "common/console_colors.h"
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

TaskDescriptor::TaskDescriptor( Action action, Action action_stemmed )
    : action_(action)
    , action_stemmed_(action_stemmed)
{
    // Create an interface
    TaskInterface task_interface;

    // Push it to the local task interfaces storage
    this->task_interfaces_.push_back( std::move( task_interface ) );
}

TaskDescriptor::TaskDescriptor( Action action, std::vector<Subject>& input_subjects ) : action_(action)
{
    // Create an interface
    TaskInterface task_interface;
    task_interface.input_subjects_ = input_subjects;

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


void TaskDescriptor::addIncompleteSubject(Subject subject)
{
    incomplete_subjects_.push_back(subject);
}


TaskInterface& TaskDescriptor::getFirstInterface()
{
    return task_interfaces_[0];
}

std::vector<Subject>& TaskDescriptor::getFirstInputSubjects()
{
    return task_interfaces_[0].input_subjects_;
}

const Action& TaskDescriptor::getAction() const
{
    return action_;
}

std::vector<TaskInterface>& TaskDescriptor::getInterfaces()
{
    return task_interfaces_;
}

bool TaskDescriptor::empty() const
{
    return task_interfaces_.empty();
}

std::vector<Subject>& TaskDescriptor::getIncompleteSubjects()
{
    return incomplete_subjects_;
}

const std::string& TaskDescriptor::getLibPath() const
{
    return task_lib_path_;
}

void TaskDescriptor::setLibPath(std::string task_lib_path)
{
    // If the string is empty then throw error
    if (task_lib_path.empty())
    {
        // TODO: THROW TEMOTO ERROR
        throw;
    }

    task_lib_path_ = task_lib_path;
}

const std::string& TaskDescriptor::getTaskClassName() const
{
    return task_class_name_;
}

void TaskDescriptor::setTaskClassName(std::string task_class_name)
{
    // If the string is empty then throw error
    if (task_class_name.empty())
    {
        // TODO: THROW TEMOTO ERROR
        throw;
    }

    task_class_name_ = task_class_name;
}

const std::string& TaskDescriptor::getTaskPackageName() const
{
    return task_package_name_;
}


std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td)
{
    stream << GREEN << "ACTION: " << td.getAction() << RESET << std::endl;

    if (!td.aliases_.empty())
    {
        stream << GREEN << "ALIASES: ";
        for (auto& alias : td.aliases_)
        {
            std::cout << alias;
            if (&alias != &td.aliases_.back())
            {
                std::cout << ", ";
            }
        }
        std::cout << RESET << std::endl;
    }

    for (auto& task_interface : td.task_interfaces_)
    {
        stream << "INTERFACE " << task_interface.id_ << " (" << task_interface.type_ << "):" << std::endl;
        stream << "|__ in __\n" << task_interface.input_subjects_;
        stream << "|__ out __\n" << task_interface.output_subjects_;
    }
    return stream;
}

}
