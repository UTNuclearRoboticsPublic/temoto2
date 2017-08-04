/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  TODO: * Fix the bug: when "terminal" task is closed, ros
 *          tries to desperately reconnect with the internal service
 *          "Retrying connection to [ajamasinII:58905] for topic [/human_chatter]"
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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

class Core
{
public:

    /**
     * @brief Core
     */
    Core()
    {
        // Subscribers
        chatter_subscriber_ = n_.subscribe("human_chatter", 1000, humanChatterCallback);

        // Initialize the classloader
        classLoader_(false);

        // Initialize the taskHandler
        taskHandler_( "core", &classLoader );

        // Create a Language Processor and initialize it by passing the list of indexed tasks.
        // Language Processor uses the information contained within the indexed tasks to detect
        // right keywords
        languageProcessor_.setTasksIndexed( taskHandler_.getIndexedTasks() );
    }

private:

    /**
     * @brief n_
     */
    ros::NodeHandle n_;

    /**
     * @brief chatter_subscriber_
     */
    ros::Subscriber chatter_subscriber_;

    /**
     * @brief languageProcessor_
     */
    LanguageProcessor languageProcessor_;

    /**
     * @brief classLoader_
     */
    class_loader::MultiLibraryClassLoader classLoader_;

    /**
     * @brief taskHandler_
     */
    TaskHandler taskHandler_;

    /**
     * @brief humanChatterCallback
     * @param my_text_in
     */
    void humanChatterCallback (std_msgs::String my_text_in)
    {
        // Process the text and get the required tasks if any
        TaskList taskList = languageProcessor_.processText(my_text_in.data);

        ROS_INFO("[core] Received %lu tasks", taskList.size());

        // Find a task
        for (auto task: taskList)
        {
            // Use the name of the task to load in the task class. task handler returns the internal
            // specific name of the task which is used in the next step.
            ROS_INFO("[core] Executing task '%s' ...", task.first.getName().c_str());

            if ( !taskHandler_.loadTask(task.first) )
            {
                continue;
                // PLEASE DO SOMETHING MORE REASONABLE
            }

            // Create an instance of the task based on the class name. a task .so file might
            // contain multiple classes, therefore path is not enough and specific name
            // must be used
            if ( taskHandler_.instantiateTask(task.first) )
            {
                //Start the task
                if ( !taskHandler_.startTask(task.first, task.second) )
                {
                    taskHandler_.stopTask(task.first.getName());
                }
            }

            std::cout << std::endl;
        }
    }
};





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
    TaskHandler taskHandler( "core", &classLoader );

    // Index the available tasks
    std::cout << "[core]: Indexing the tasks ..." << std::endl;
    boost::filesystem::directory_entry dir("/home/robert/catkin_ws/src/temoto2/tasks/");
    taskHandler.indexTasks(dir, 1);

    // Create a Language Processor and initialize it by passing the list of indexed tasks.
    // Language Processor uses the information contained within the indexed tasks to detect
    // right keywords
    LanguageProcessor languageProcessor;
    languageProcessor.setTasksIndexed( taskHandler.getIndexedTasks() );

    // Find something from the indexed tasks
    std::cout << "[core]: Looking for 'terminal' ..." << std::endl;

    for (TaskInfo taskInfoInst : taskHandler.findTaskLocal("terminal"))
    {
        std::cout << "found:" << taskInfoInst << std::endl;
    }

    // Check for errors
    if( taskHandler.errorHandler_.gotUnreadErrors() )
    {
        std::cout << "[core]: Printing the errorstack:" << taskHandler.errorHandler_;
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();

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
                ROS_INFO("[core] Executing task '%s' ...", task.first.getName().c_str());

                if ( !taskHandler.loadTask(task.first) )
                {
                    continue;
                    // PLEASE DO SOMETHING MORE REASONABLE
                }

                // Create an instance of the task based on the class name. a task .so file might
                // contain multiple classes, therefore path is not enough and specific name
                // must be used
                if ( taskHandler.instantiateTask(task.first) )
                {
                    //Start the task
                    if ( !taskHandler.startTask(task.first, task.second) )
                    {
                        taskHandler.stopTask(task.first.getName());
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

