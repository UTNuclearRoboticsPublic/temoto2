#include "ros/ros.h"
#include <class_loader/class_loader.h>
#include <class_loader/multi_library_class_loader.h>
#include "std_msgs/String.h"
#include <boost/any.hpp>
#include <ctype.h>

#include <sstream>
#include "base_task/task.h"
#include "core/language_processor/language_processor.h"
#include "core/task_handler/task_handler.h"
#include "core/task_handler/description_processor.h"

#include "core/task_handler/task_info.h"

bool msgReceived = false;
std::string my_text;

void humanChatterCallback (std_msgs::String my_text_in)
{
    msgReceived = true;
    my_text = my_text_in.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temoto2_core");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber chatter_subscriber = n.subscribe("human_chatter", 1000, humanChatterCallback);

    // Create a class loader instance and pass it to the taskhandler. For some reason
    // I could not create a private classloader inside the taskhandler, hence it is being
    // passed over from here
    class_loader::MultiLibraryClassLoader classLoader(false);
    TaskHandler taskHandler( &classLoader );

    // Index the available tasks
    std::cout << "[core]: Indexing the tasks ..." << std::endl;
    boost::filesystem::directory_entry dir("/home/robert/catkin_ws/src/temoto2/tasks/");
    taskHandler.indexTasks(dir, 1);

    // Create a Panguage Processor and initialize it by passing the list of indexed tasks.
    // Language Processor uses the information contained within the indexed tasks to detect
    // right keywords
    LanguageProcessor languageProcessor;
    languageProcessor.setTasksIndexed( taskHandler.getIndexedTasks() );

    // Find something from the indexed tasks
    std::cout << "[core]: Looking for 'terminal' ..." << std::endl;

    for (TaskInfo taskInfoInst : taskHandler.findTask("terminal"))
    {
        std::cout << "found:" << taskInfoInst << std::endl;
    }

    // Check for errors
    if( taskHandler.errorHandler_.gotUnreadErrors() )
    {
        std::cout << "[core]: Printing the errorstack:" << taskHandler.errorHandler_;
    }

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (msgReceived)
        {
            // Process the text and get the required tasks if any
            TaskList taskList = languageProcessor.processText(my_text);

            ROS_INFO("[core] Received %lu tasks", taskList.size());

            // Find a task
            for (auto task: taskList)
            {
                // Use the name of the task to load in the task class. task handler returns the internal
                // specific name of the task which is used in the next step.
                ROS_INFO("[core] Executing task '%s' ...", task.first.c_str());

                std::string taskClassName = taskHandler.loadTask(task.first);
                if ( taskClassName.empty() )
                {
                    continue;
                }

                // Create an instance of the task based on the class name. a task .so file might
                // contain multiple classes, therefore path is not enough and specific name
                // must be used
                if ( !taskHandler.instantiateTask(taskClassName) )
                {
                    //Start the task
                    if ( taskHandler.startTask(taskClassName, task.second) )
                    {
                        taskHandler.stopTask(taskClassName);
                    }
                }

                std::cout << std::endl;
            }

            msgReceived = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

