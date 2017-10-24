#include "TTP/task_container.h"

namespace TTP
{

// Constructor
TaskContainer::TaskContainer(boost::shared_ptr<Task> task_pointer,TaskInterface task_interface)
    : task_pointer_(task_pointer), task_interface_(task_interface)
{}

Subjects TaskContainer::operator()(Subjects input_subjects)
{
    // Create a copy of input subjects
    std::vector<Subject> input_subjects_c = input_subjects;

    /*
     * Check for incomplete local subjects. Get the missing information for the
     * incomplete subjects from input subjects
     */
    for (auto& l_sub : task_interface_.input_subjects_)
    {
        if (l_sub.is_complete_)
        {
            continue;
        }

        // go through the input subjects
        for (unsigned int i=0; i<input_subjects_c.size(); i++)
        for (auto i_sub_it = input_subjects_c.begin(); i_sub_it != input_subjects_c.end(); ++i_sub_it)
        {
            // Check type
            if (l_sub.type_ != i_sub_it->type_)
            {
                continue;
            }

            // Check data
            if (l_sub.data_ != i_sub_it->data_)
            {
                continue;
            }

            // At this point we have a match. Make a copy and delete the matching entry
            l_sub = *i_sub_it;
            input_subjects_c.erase(i_sub_it);
            break;
        }
    }

    // Start the task
    task_pointer_->startTask(task_interface_);

    // Return the solution
    return task_pointer_->getSolution();
}
}// END of TTP namespace
