/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *		* Demonstrate dynamic subscription
 *              * Brings up a terminal that allows to send commands to the core
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/task.h"                 				 // The base task
#include <class_loader/class_loader.h>                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "context_manager/human_context/human_context_interface.h"

// First implementaton
class TaskShow: public TTP::Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskShow()
    {
        // Do something here if needed
        ROS_INFO("TaskShow constructed");
    }

    // startTask with arguments
    bool startTask(TTP::TaskInterface task_interface)
    {
        std::cout << "Started the start task, returning\n";
        unsigned int intrface_id = task_interface.id_;
        std::vector<TTP::Subject> input_subjects = task_interface.input_subjects_;
        for(auto& i_sub : input_subjects)
        {
            std::cout << i_sub << std::endl;
        }
        
        return true;
    }

    std::string getStatus()
    {
        std::string str = "healthy";
        return str;
    }

    std::vector<TTP::Subject> getSolution()
    {
        // Construct an empty vector
        std::vector<TTP::Subject> return_subjects;

        return return_subjects;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    ~TaskShow()
    {
        ROS_INFO("TaskShow destructed");
    }

private:

    // Human context interface object
    // HumanContextInterface <TaskShow> hci_;

    /**
     * @brief class_name_
     */
    std::string class_name_ = "TaskShow";

    int numberOfArguments = 1;
    std::string arg_0;

    bool print_ = true;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskShow, TTP::Task);
