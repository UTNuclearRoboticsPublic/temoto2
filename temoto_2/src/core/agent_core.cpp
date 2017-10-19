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
#include "std_msgs/String.h"
#include <ctype.h>
#include <sstream>

#include "core/common.h"
#include "core/language_processor/language_processor.h"
#include "core/task_handler/task_handler.h"

// TBB test
#include <cstdio>
#include "tbb/flow_graph.h"

using namespace tbb::flow;

struct body {
    std::string my_name;
    body( const char *name ) : my_name(name) {}
    void operator()( continue_msg ) const {
        printf("%s\n", my_name.c_str());
    }
};


class TemotoCore
{
public:

    /**
     * @brief Core
     */
    TemotoCore( std::string name ) : task_handler_( name )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        ROS_INFO("[TemotoCore::TemotoCore]: Construncting the core ...");

        // Subscribers
        chatter_subscriber_ = n_.subscribe("human_chatter", 1000, &TemotoCore::humanChatterCallback, this);

        try
        {
            /*
             * Index (look recursivey for the tasks in the given folder up to specified depth)
             * the available tasks, otherwise the task handler would have no clue about the available
             * tasks. Later the indexing could be via indexing task.
             * TODO: Read the base path from the parameter server
             */
            ROS_INFO("[TemotoCore::TemotoCore]: Indexing the tasks ...");

            std::string home = boost::filesystem::path(getenv("HOME")).string();
            boost::filesystem::directory_entry dir(home + "/catkin_ws/src/temoto2/tasks/");

            task_handler_.indexTasks(dir, 1);

            /*
             * Create a Language Processor and initialize it by passing the list of indexed tasks.
             * Language Processor uses the information contained within the indexed tasks to detect
             * right keywords (tasks and their I/O arguments). Since the list of indexed tasks are
             * passed as a pointer, the "TaskHandler::indexTasks" call will be enough to keep the
             * Language Processor up-to-date
             */
            ROS_INFO("[TemotoCore::TemotoCore]: Initializing the Language Processor ...");
            language_processor_.setTasksIndexed( task_handler_.getIndexedTasks() );
        }
        catch( error::ErrorStackUtil & e )
        {
            // Rethrow the error
            e.forward( prefix );\
            throw e;
        }
    }

private:

    /**
     * @brief n_
     */
    ros::NodeHandle n_;

    /**
     * @brief language_processor_
     */
    LanguageProcessor language_processor_;

    /**
     * @brief task_handler_
     */
    TaskHandler task_handler_;

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    /**
     * @brief class_name_
     */
    std::string class_name_ = "TemotoCore";

    /**
     * @brief chatter_subscriber_
     */
    ros::Subscriber chatter_subscriber_;

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
            task_handler_.executeTask(task.first, task.second);
            std::cout << std::endl;
        }

        // Check for errors
        if( task_handler_.error_handler_.gotUnreadErrors() )
        {
            std::cout << "[TemotoCore::humanChatterCallback]: Printing the errorstack:" << task_handler_.error_handler_;
        }
    }
};


int main(int argc, char **argv)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", "Core", __func__);

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

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    // Create the Core object
    TemotoCore* temoto_core;

    try
    {
        temoto_core = new TemotoCore("core");
    }
    catch( error::ErrorStackUtil & e )
    {
        // Append the error to local ErrorStack
        error_handler_.append(e);

        ROS_ERROR("%s Failed to start the core", prefix.c_str());
        return 1;
    }

    // Create async spinner, otherwise there is a possibility of locking during core calls
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Rate loop_rate(5);

    /*
     * Set the initial tasks
     * TODO: currently done just by publishing a chatter msg, do it
     *       in a reasonable manner by just loading in the tasks.
     */
    std_msgs::String init_msg;
    init_msg.data = "terminal";

    chatter_publisher.publish(init_msg);

    ROS_INFO("%s Core is up and running", prefix.c_str());

    /*
     * TBB TESTS
     * from: https://software.intel.com/en-us/node/506216
     */
/*
    std::cout << "INTEL TBB TESTS" << std::endl;

    graph g;

    broadcast_node< continue_msg > start(g);
    continue_node<continue_msg> a( g, body("A"));
    continue_node<continue_msg> b( g, body("B"));
    continue_node<continue_msg> c( g, body("C"));
    continue_node<continue_msg> d( g, body("D"));
    continue_node<continue_msg> e( g, body("E"));

    make_edge( start, a );
    make_edge( start, b );
    make_edge( a, c );
    make_edge( b, c );
    make_edge( c, d );
    make_edge( a, e );
*/
/*
    graph g;
    continue_node< continue_msg> hello( g,
      []( const continue_msg &) {
          std::cout << "Hello";
      }
    );
    continue_node< continue_msg> world( g,
      []( const continue_msg &) {
          std::cout << " World\n";
      }
    );
    make_edge(hello, world);
*/
    while (ros::ok())
    {
        /*
         * Do something
         */
/*
        hello.try_put(continue_msg());
        g.wait_for_all();
*/
/*
        start.try_put( continue_msg() );
        g.wait_for_all();
*/
//        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

