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
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "temoto_2/StopTask.h"

// First implementaton
class TaskStop: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaskStop()
{
    // Do something here if needed
    stop_task_client_ = n_.serviceClient<temoto_2::StopTask>("temoto_agent/stop_task");
    ROS_INFO("TaskStop constructed");
}

// startTask
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

    // Interface 1
    case 1:
        startInterface_1();
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
    TTP::Subject action_0_in = TTP::getSubjectByType("action", input_subjects);
    std::string  action_0_word_in = action_0_in.words_[0];

    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string  what_0_word_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskStop: Stopping '" << action_0_word_in << "' with arg '" << what_0_word_in << "'\n";
    what_0_word_out = what_0_word_in;

    temoto_2::StopTask stop_task_srv;
    stop_task_srv.request.action = action_0_word_in;
    stop_task_srv.request.what = what_0_word_in;

    // Call the server
    stop_task_client_.call(stop_task_srv);

    ROS_INFO("[TaskStop::startTask] '/core/stop_task' service respinded: %s", stop_task_srv.response.message.c_str());

    // Check the result
    if (stop_task_srv.response.code != 0)
    {
        // TODO: do something
    }


// --------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
}

/*
 * Interface 1 body
 */
void startInterface_1()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string  what_0_word_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskStop: Stopping '" << what_0_word_in << "'\n";
    what_0_word_out = what_0_word_in;

    temoto_2::StopTask stop_task_srv;
    stop_task_srv.request.what = what_0_word_in;

    // Call the server
    stop_task_client_.call(stop_task_srv);

    ROS_INFO("[TaskStop::startTask] '/core/stop_task' service respinded: %s", stop_task_srv.response.message.c_str());

    // Check the result
    if (stop_task_srv.response.code == 0)
    {
        //bla
    }

// --------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
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
    return output_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

~TaskStop()
{
    std::cout << "TaskStop destructed" << std::endl;
}

private:

// Human context interface object
// HumanContextInterface <TaskStop> hci_;

/**
 * @brief class_name_
 */
std::string class_name_ = "TaskStop";

ros::NodeHandle n_;
ros::ServiceClient stop_task_client_;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskStop, TTP::BaseTask);
