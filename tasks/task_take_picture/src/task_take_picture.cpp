/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that commands a robot to take picture with its arm-mounted camera.
 *
 *	TASK DESCRIPTION:
 *      * Take picture with arm-mounted camera using the following possible sources:
 *         -- head orientation
 *         -- hand position
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"             // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "context_manager/human_context/human_context_interface.h"

// First implementaton
class TaskTakePicture: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskTakePicture()
    {
        // Do something here if needed
        ROS_INFO("TaskTakePicture constructed");
    }

    // startTask without arguments
    bool startTask()
    {
        // Name of the method, used for making debugging a bit simpler
        const std::string method_name_ = "startTask";
        std::string prefix = formatMessage("", this->class_name_, method_name_);

        // Build a gesture specifier
        // TODO: This shoud be done via speech specifier helper class
        std::vector <temoto_2::speechSpecifier> speechSpecifiers;
        temoto_2::speechSpecifier speechSpecifier;
        speechSpecifier.dev = "device";
        speechSpecifier.type = "text";

        speechSpecifiers.push_back(speechSpecifier);


        // Subscribe to gesture topic
        try
        {
            hci_.getSpeech(speechSpecifiers, &TaskTakePicture::speech_callback, this );
            true;
        }

        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            this->error_handler_.append(e);
        }

        return true;
    }

    // startTask with arguments
    bool startTask(int subtaskNr, std::vector<boost::any> arguments )
    {
        // Check if arguments vector contains expected amount of arguments
        if (arguments.size() != numberOfArguments)
        {
            std::cerr << "[TaskTakePicture/startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return false;
        }

        // If it does, try to cast the arguments
        try
        {
            arg0 = boost::any_cast<int>(arguments[0]);
            arg1 = boost::any_cast<std::string>(arguments[1]);
            arg2 = boost::any_cast<double>(arguments[2]);

            // Print them out
            std::cout << arg0 << '\n';
            std::cout << arg1 << '\n';
            std::cout << arg2 << '\n';

            return true;
        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[TaskTakePicture::startTask]: " << e.what() << '\n';
            return false;
        }
    }

    bool stopTask()
    {
        return true;
    }

    std::string getStatus()
    {
        std::string str = "healthy";
        return str;
    }

    std::vector<boost::any> getSolution( int subtaskNr )
    {
        // Construct an empty vector
        std::vector<boost::any> solutionVector;

        // Check the subtask number
        if ( subtaskNr == 0)
        {
            boost::any retArg0 = arg0;
            boost::any retArg1 = arg1;
            boost::any retArg2 = arg2;

            solutionVector.push_back(retArg0);
            solutionVector.push_back(retArg1);
            solutionVector.push_back(retArg2);
        }

        return solutionVector;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    // Callback for processing gestures
    void speech_callback(std_msgs::String msg)
    {
        //ROS_INFO("Speech callback got: %s", msg.data.c_str());
    }

    ~TaskTakePicture()
    {
        ROS_INFO("[TaskTakePicture::~TaskTakePicture]TaskTakePicture destructed");
    }

private:

    /**
     * @brief class_name_
     */
    std::string class_name_ = "TaskTakePicture";

    // Human context interface object
    HumanContextInterface <TaskTakePicture> hci_;

    int numberOfArguments = 3;

    int arg0;
    std::string arg1;
    double arg2;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskTakePicture, Task);
