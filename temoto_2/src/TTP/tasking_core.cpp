#include "TTP/tasking_core.h"
#include "common/tools.h"
#include "common/console_colors.h"
#include "common/temoto_log_macros.h"
#include "base_error/base_error.h"
#include <cstdio>
#include "tbb/flow_graph.h"

namespace TTP
{

TaskingCore::TaskingCore(std::string node_name)
    : task_manager_(node_name)
{
    try
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = common::generateLogPrefix(node_name, class_name_, __func__);

        std::string temoto_path = ros::package::getPath(ROS_PACKAGE_NAME);

        /*
         * Index (look recursivey for the tasks in the given folder up to specified depth)
         * the available tasks, otherwise the task handler would have no clue about the available
         * tasks. Later the indexing could be via indexing task.
         * TODO: Read the base path from the parameter server
         */
        std::cout << prefix << " Indexing the tasks ... " << std::flush;
        boost::filesystem::directory_entry dir(temoto_path + "/../tasks");
        task_manager_.indexTasks(dir, 1);
        std::cout << "done\n";

        std::vector <TTP::TaskDescriptor>& tds = task_manager_.getIndexedTasks();

        // Print out the tasks
        std::cout << "Found " << tds.size() << " tasks. Printing ...\n";
        for (auto& td : tds)
        {
            std::cout << td << std::endl;
        }

        /*
         * Initialize the language processor by giving it the path to language model files
         */
        language_processor_ = new MetaLP(temoto_path + "/include/TTP/language_processors/meta/models/");

        /*
         * Subscribe to human chatter topic. This triggers the callback that processes text
         * messages and trys to find and execute tasks based on the text
         */
        human_chatter_subscriber_ = nh_.subscribe("/temoto_2/human_chatter", 1, &TaskingCore::humanChatterCb, this);
    }
    catch (error::ErrorStackUtil& e)
    {
        // Rethrow or do whatever
        //std::cout << e.getStack();
    }
}

void TaskingCore::humanChatterCb (std_msgs::String chat)
{
    try
    {
        std::cout << BOLDWHITE << "Received: " << chat.data << RESET << std::endl << std::endl;

        // Process the text and receive a task tree
        TaskTree tt = language_processor_->processText(std::move(chat.data));

        // Print out the task tree
        std::cout << "Task Tree: " << tt;

        /*
         * TODO: There is a massive flaw somewhere in tbb, ros or in me. This flow graph
         * object should not exist here, instead it should be packed inside the "executeTaskTree"
         * function, along with the task tree. But with the mentioned configuration, the process
         * just (randomly) hangs after returning from "executeTaskTree" ... If the flow graph object
         * is destructed in this scope, all works just fine.
         */
        tbb::flow::graph flow_graph;

        // Execute the tree
        task_manager_.executeTaskTree (tt.getRootNode(), flow_graph);
    }
    catch (error::ErrorStackUtil& e)
    {
        // Rethrow or do whatever
        //std::cout << e.getStack();
    }

    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * \n\n";
}

}// END of TTP namespace

