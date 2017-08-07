/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  TODO: * Fix the bug: when "terminal" task is closed, ros
 *          tries to desperately reconnect with the internal service:
 *          "Retrying connection to [ajamasinII:58905] for topic [/human_chatter]"
 *          This is visible when the debug logger level is enabled.
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

class TemotoCore
{
public:

    /**
     * @brief Core
     */
    TemotoCore()
    {
        ROS_INFO("[TemotoCore::TemotoCore]: Construncting the core ...");

        // Subscribers
        chatter_subscriber_ = n_.subscribe("human_chatter", 1000, &TemotoCore::humanChatterCallback, this);

        // Initialize the taskHandler. Pass in the name of the subsystem
        // (used for namespacing services and debug information)
        task_handler_ = new TaskHandler("core");

        // Index (look recursivey for the tasks in the given folder up to specified depth)
        // the available tasks, otherwise the task handler would have no clue about the available
        // tasks. Later the indexing could be via indexing task.
        // TODO: Read the base path from the parameter server
        ROS_INFO("[TemotoCore::TemotoCore]: Indexing the tasks ...");
        boost::filesystem::directory_entry dir("/home/robert/catkin_ws/src/temoto2/tasks/");
        task_handler_->indexTasks(dir, 1);

        // Create a Language Processor and initialize it by passing the list of indexed tasks.
        // Language Processor uses the information contained within the indexed tasks to detect
        // right keywords (tasks and their I/O arguments)
        ROS_INFO("[TemotoCore::TemotoCore]: Initializing the Language Processor ...");
        language_processor_.setTasksIndexed( task_handler_->getIndexedTasks() );
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
     * @brief language_processor_
     */
    LanguageProcessor language_processor_;

    /**
     * @brief task_handler_
     */
    TaskHandler* task_handler_;

    /**
     * @brief humanChatterCallback
     * @param my_text_in
     */
    void humanChatterCallback (std_msgs::String my_text_in)
    {
        // Process the text and get the required tasks if any
        TaskList taskList = language_processor_.processText(my_text_in.data);

        ROS_INFO("[TemotoCore::humanChatterCallback] Received %lu tasks", taskList.size());

        // Find a task
        for (auto task: taskList)
        {
            // Use the name of the task to load in the task class. task handler returns the internal
            // specific name of the task which is used in the next step.
            ROS_INFO("[TemotoCore::humanChatterCallback] Executing task '%s' ...", task.first.getName().c_str());

            if ( !task_handler_->loadTask(task.first) )
            {
                continue;
                // PLEASE DO SOMETHING MORE REASONABLE
            }

            // Create an instance of the task based on the class name. a task .so file might
            // contain multiple classes, therefore path is not enough and specific name
            // must be used
            if ( task_handler_->instantiateTask(task.first) )
            {
                //Start the task
                if ( !task_handler_->startTask(task.first, task.second) )
                {
                    task_handler_->stopTask(task.first.getName());
                }
            }

            std::cout << std::endl;
        }

        // Check for errors
        if( task_handler_->error_handler_.gotUnreadErrors() )
        {
            std::cout << "[TemotoCore::humanChatterCallback]: Printing the errorstack:" << task_handler_->error_handler_;
        }
    }
};


int main(int argc, char **argv)
{
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * " << std::endl;
    std::cout << "*                                                     * " << std::endl;
    std::cout << "*                       TEMOTO CORE                   * " << std::endl;
    std::cout << "*                          v.1.0                      * " << std::endl;
    std::cout << "*                                                     * " << std::endl;
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * " << std::endl;
    std::cout << "\n\n";

    ros::init(argc, argv, "temoto_core");
    ros::NodeHandle n;

    // Publisher for publishing messages to core itself
    ros::Publisher chatter_publisher = n.advertise<std_msgs::String>("human_chatter", 1000);

    // Create the Core object
    TemotoCore temoto_core;

    // Create async spinner, otherwise there is a possibility
    // of locking during core calls
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Rate loop_rate(10);

    // Set the initial tasks
    // TODO: currently done just by publishing a chatter msg, do it
    //       in a reasonable manner by just loading in the tasks.
    std_msgs::String init_msg;
    init_msg.data = "terminal";
    chatter_publisher.publish(init_msg);

    ROS_INFO("[temoto_core] Core is up and running");

    while (ros::ok())
    {
        /*
         * Do something
         */
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

