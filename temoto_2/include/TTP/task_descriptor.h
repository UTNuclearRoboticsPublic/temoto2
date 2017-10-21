#ifndef TASK_DESCRIPTOR_H
#define TASK_DESCRIPTOR_H

#include "TTP/io_descriptor.h"
//#include "TTP/base_task/base_task.h"

namespace TTP
{

typedef std::string Action;

struct TaskInterface
{
    std::vector<Subject> input_subjects_;
    std::vector<Subject> output_subjects_;
};

class TaskDescriptorProcessor;
class TaskTreeBuilder;

/**
 * @brief This class contains all the information needed for finding, loading
 * and executing a given task.
 */
class TaskDescriptor
{
    friend TaskDescriptorProcessor;
    friend TaskTreeBuilder;

public:

    TaskDescriptor(){}

    TaskDescriptor( Action action );

    TaskDescriptor( Action action, std::vector<Subject>& input_subjects );

    TaskDescriptor( Action action, TaskInterface& task_interface );

    TaskDescriptor( Action action, std::vector<TaskInterface>& task_interfaces );

    friend std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td);


    void addIncompleteSubject(Subject subject);


    const Action& getAction() const;

    std::vector<TaskInterface>& getInterfaces();

    std::vector<Subject>& getFirstInputSubjects();

    std::vector<Subject>& getIncompleteSubjects();

    bool empty();

private:
    std::vector<TaskInterface> task_interfaces_;

    Action action_;

    std::string task_class_name_;

    std::string task_package_name_;

    std::string task_lib_path_;

    std::vector<Subject> incomplete_subjects_;

    //boost::shared_ptr<Task> task_pointer_;
};
}

#endif
