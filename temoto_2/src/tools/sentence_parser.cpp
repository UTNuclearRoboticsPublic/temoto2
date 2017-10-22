#include "ros/ros.h"

#include "TTP/language_processors/meta/meta_lp.h"
#include "TTP/task_descriptor_processor.h"
#include "TTP/task_manager.h"

#include "base_error/base_error.h"

//using namespace TTP;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentence_parser");
    ros::NodeHandle nh;

    // Task manager object
    TTP::TaskManager task_manager("Tester");

    try
    {
        /*
         * Index (look recursivey for the tasks in the given folder up to specified depth)
         * the available tasks, otherwise the task handler would have no clue about the available
         * tasks. Later the indexing could be via indexing task.
         * TODO: Read the base path from the parameter server
         */
        std::cout << "Indexing the tasks ... " << std::flush;

        std::string home = boost::filesystem::path(getenv("HOME")).string();
        boost::filesystem::directory_entry dir(home + "/catkin_ws/src/temoto2/tasks/");

        task_manager.indexTasks(dir, 1);

        std::cout << "done\n";

        std::vector <TTP::TaskDescriptor>& tds = task_manager.getIndexedTasks();

        // Print out the tasks
        std::cout << "Found " << tds.size() << " tasks. Printing ...\n";
        for (auto& td : tds)
        {
            std::cout << td << std::endl;
        }
    }
    catch (error::ErrorStackUtil& e)
    {
        // Rethrow or do whatever
        //std::cout << e.getStack();
    }

    TTP::MetaLP language_processor("/home/robert/repos_sdks/meta/models/");

    std::string line;

    while (ros::ok())
    {
        std::cout << " > ";
        std::getline(std::cin, line);

        try
        {
            // Process the text and receive a task tree
            TTP::TaskTree tt = language_processor.processText(std::move(line));

            // Print out the task tree
            std::cout << "Task Tree: " << tt;

            // Find connecting tasks
            std::vector<TTP::Subject> empty_subs; // stupid hack
            task_manager.connectTasks(tt.getRootNode(), empty_subs);

            // Print task tree task descriptors
            tt.printTaskDescriptors(tt.getRootNode());
        }
        catch (error::ErrorStackUtil& e)
        {
            // Rethrow or do whatever
            //std::cout << e.getStack();
        }

        std::cout << "----------------------------------------------------------\n";

        ros::Duration(0.5).sleep();
    }

    return 0;
}
