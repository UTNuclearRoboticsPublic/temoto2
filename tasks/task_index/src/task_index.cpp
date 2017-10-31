/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that calls the "index_tasks" service of the Task Handler
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                  				 // The base task
#include <class_loader/class_loader.h>               // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "ros/package.h"
#include "temoto_2/IndexTasks.h"


// First implementaton
class TaskIndex: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaskIndex()
{
    index_tasks_client_ = n_.serviceClient<temoto_2::IndexTasks>("temoto_agent/index_tasks");
    ROS_INFO("TaskIndex constructed");
}

/*
 * startTask
 */
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

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskIndex: Indexing the tasks '" << what_0_word_in << "'\n";

    // Create a service message
    temoto_2::IndexTasks index_tasks_srv;
    index_tasks_srv.request.directory = ros::package::getPath(ROS_PACKAGE_NAME) + "/..";
    //index_tasks_srv.request.directory = "/home/robert/catkin_ws/src/temoto2/temoto_2/../tasks";
    // Call the server
    index_tasks_client_.call(index_tasks_srv);

    ROS_INFO("[TaskIndex::startTask] '/temoto_core/index_tasks' service responded: %s", index_tasks_srv.response.message.c_str());

    // Check the result
    if (index_tasks_srv.response.code != 0)
    {
        // TODO: do something
    }
// --------------------------------</ USER CODE >-------------------------------
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

~TaskIndex()
{
    ROS_INFO("[TaskIndex::~TaskIndex] TaskIndex destructed");
}

private:

    ros::NodeHandle n_;
    ros::ServiceClient index_tasks_client_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskIndex, TTP::BaseTask);
