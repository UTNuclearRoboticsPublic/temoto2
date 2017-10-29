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
// * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;

    switch(task_interface.id_)
    {
    // Interface 0
    case 0:
        startInterface_0();
    break;
    }

    return true;
// * AUTO-GENERATED, DO NOT MODIFY *
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];
    std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);

    TTP::Subject where_0_in = TTP::getSubjectByType("where", input_subjects);
    std::string  where_0_word_in = where_0_in.words_[0];

    // Creating output variables
    std::string  what_0_word_out;
    std::string  what_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskShow: Showing '" << what_0_word_in << "' in '" << where_0_word_in  <<  "' @ '" << what_0_data_0_in << "' topic\n";
    what_0_word_out = what_0_word_in;
    what_0_data_0_out = what_0_data_0_in;

// --------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
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
    std::cout << "TaskShow destructed" << std::endl;
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
