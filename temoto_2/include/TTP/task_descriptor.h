#ifndef TASK_DESCRIPTOR_H
#define TASK_DESCRIPTOR_H

#include "TTP/io_descriptor.h"

namespace TTP
{

class TaskDescriptorProcessor;
class TaskTreeBuilder;
class TaskManager;

typedef std::string Action;

struct TaskInterface
{
    unsigned int id_;
    std::string type_;
    std::string alias_;
    std::vector<Subject> input_subjects_;
    std::vector<Subject> output_subjects_;
};

/**
 * @brief This class contains all the information needed for finding, loading
 * and executing a given task.
 */
class TaskDescriptor
{
    friend TaskDescriptorProcessor;
    friend TaskTreeBuilder;
    friend TaskManager;

public:

    TaskDescriptor(){}

    TaskDescriptor( Action action );

    TaskDescriptor( Action action, Action action_stemmed );

    TaskDescriptor( Action action, std::vector<Subject>& input_subjects );

    TaskDescriptor( Action action, TaskInterface& task_interface );

    TaskDescriptor( Action action, std::vector<TaskInterface>& task_interfaces );

    friend std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td);


    void addIncompleteSubject(Subject subject);


    const Action& getAction() const;

    std::vector<TaskInterface>& getInterfaces();

    std::vector<Subject>& getFirstInputSubjects();

    TaskInterface& getFirstInterface();

    std::vector<Subject>& getIncompleteSubjects();


    const std::string& getLibPath() const;

    void setLibPath(std::string task_lib_path);


    const std::string& getTaskClassName() const;

    void setTaskClassName(std::string task_class_name);


    void setFirstOutputSubjects(Subjects output_subjects);

    const std::string& getTaskPackageName() const;


    bool empty() const;

private:

    std::vector<TaskInterface> task_interfaces_;

    Action action_;

    Action action_stemmed_;

    std::vector<Action> aliases_;

    std::vector<Action> aliases_stemmed_;

    std::string task_class_name_;

    std::string task_package_name_;

    std::string task_lib_path_;

    std::vector<Subject> incomplete_subjects_;

};
}

#endif
