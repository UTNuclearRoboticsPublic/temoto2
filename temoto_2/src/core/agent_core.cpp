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
    ros::init(argc, argv, "teleop_core");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber chatter_subscriber = n.subscribe("human_chatter", 1000, humanChatterCallback);

    class_loader::MultiLibraryClassLoader classLoader(false);
    TaskHandler taskHandler( &classLoader );

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (msgReceived)
        {
            boost::filesystem::directory_entry dir("/home/robert/catkin_ws/src/tark_temoto/tasks/");

            std::vector<TaskInfo> taskInfo = taskHandler.findTask(dir, "add", 1);
            for (TaskInfo taskInfoInst : taskInfo)
            {
                std::cout << "found: " << taskInfoInst.getName() << std::endl;
            }

 /*
            // Get tasks
            TaskList taskList;
            taskList = langProc.processText(my_text);

            ROS_INFO("[agent_core] Received %lu tasks", taskList.size());

            // Create task handling object
            class_loader::MultiLibraryClassLoader classLoader(false);
            TaskHandler taskHandler( &classLoader );

            // Find a task
            for (auto task: taskList)
            {
                // Get the path to the required task since the librarys are loaded based on
                // the path. A task keyword might return multiple librarys, hence the "map"
                // that is returned by "findtask" method
                std::map<std::string, std::string> taskToPath = taskHandler.findTask(task.first);
                if ( taskToPath.empty() )
                {
                    continue;
                }

                ROS_DEBUG("[agent_core] task '%s' was found at '%s'", task.first.c_str(), taskToPath.at(task.first).c_str() );

                //
                //    DO SOME MAGIC HERE IF NEEDED, choose a right task lib or whatever
                //

                // Use the path to load in the task class. task handler returns the internal
                // specific name of the task which is used in the next step.
                std::string taskClassName = taskHandler.loadTask(taskToPath.at(task.first));
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
                    if ( !taskHandler.startTask(taskClassName, task.second) )
                    {
                        taskHandler.stopTask(taskClassName);
                    }
                }

                std::cout << std::endl;
            }
 */
            msgReceived = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

