#include "ros/ros.h"
#include "ros/package.h"

#include "temoto_nlp/language_processors/meta/meta_lp.h"
#include "temoto_nlp/task_descriptor_processor.h"
#include "temoto_nlp/task_manager.h"

#include "temoto_core/temoto_error/temoto_error.h"

// TBB test
#include <cstdio>
#include "tbb/flow_graph.h"

//using namespace temoto_nlp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentence_parser");
    ros::NodeHandle nh;
    std::string temoto_path = ros::package::getPath(ROS_PACKAGE_NAME);

    // Task manager object
    temoto_nlp::TaskManager task_manager("temoto_core");

    try
    {
        /*
         * Index (look recursivey for the tasks in the given folder up to specified depth)
         * the available tasks, otherwise the task handler would have no clue about the available
         * tasks. Later the indexing could be via indexing task.
         * TODO: Read the base path from the parameter server
         */
        std::cout << "Indexing the tasks ... " << std::flush;

        boost::filesystem::directory_entry dir(temoto_path + "/../tasks");

        task_manager.indexTasks(dir, 1);

        std::cout << "done\n";
/*
        std::vector <temoto_nlp::TaskDescriptor>& tds = task_manager.getIndexedTasks();

        // Print out the tasks
        std::cout << "Found " << tds.size() << " tasks. Printing ...\n";
        for (auto& td : tds)
        {
            std::cout << td << std::endl;
        }
*/
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    temoto_nlp::MetaLP language_processor(temoto_path + "/include/TTP/language_processors/meta/models/");
    std::string line;

    ros::AsyncSpinner spinner(0); // TODO: does not need an async spinner
    spinner.start();

    while (ros::ok())
    {
        std::cout << " > " << std::flush;
        std::getline(std::cin, line);

        try
        {
            // Process the text and receive a task tree
            temoto_nlp::TaskTree tt = language_processor.processText(std::move(line));

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
            task_manager.executeTaskTree (tt.getRootNode(), flow_graph);

        }
        catch (temoto_core::error::ErrorStack& error_stack)
        {
          FORWARD_ERROR(error_stack);
        }

        std::cout << "----------------------------------------------------------\n" << std::flush;

        //ros::Duration(1).sleep();
    }

    ros::waitForShutdown();

    return 0;
}
