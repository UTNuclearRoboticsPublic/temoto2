#include "ros/ros.h"

#include "TTP/language_processors/meta/meta_lp.h"
#include "TTP/task_descriptor_processor.h"
#include "TTP/task_manager.h"

#include "base_error/base_error.h"

// TBB test
#include <cstdio>
#include "tbb/flow_graph.h"

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
    TTP::TaskTree tt;
    std::string line;

    while (ros::ok())
    {
        std::cout << " > ";
        std::getline(std::cin, line);

        try
        {
            // Process the text and receive a task tree
            tt = language_processor.processText(std::move(line));

            // Print out the task tree
            std::cout << "Task Tree: " << tt;

            // Find connecting tasks
            std::vector<TTP::Subject> empty_subs; // stupid hack
            task_manager.connectTaskTree(tt.getRootNode(), empty_subs);

            // Print task tree task descriptors
            tt.printTaskDescriptors(tt.getRootNode());

            // Load and initialize the tasks
            task_manager.loadAndInitializeTaskTree(tt.getRootNode());

            // Create a tbb flow graph
            std::cout << "\n TBB business \n";
            tbb::flow::graph flow_graph;
            task_manager.makeFlowGraph(tt.getRootNode(), flow_graph);
            task_manager.connectFlowGraph(tt.getRootNode());

            // Start the flow graph
            TTP::Subjects dummy_subjects;
            tt.getRootNode().root_fgn_->try_put(dummy_subjects);
            flow_graph.wait_for_all();
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
