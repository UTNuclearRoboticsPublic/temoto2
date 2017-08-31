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
#include "temoto_2/startSensorRequest.h"
#include "output_manager/output_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"

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

        return true;
    }

    // startTask with arguments
	bool startTask(int subtaskNr, std::vector<boost::any> arguments )
	{

		// Check if arguments vector contains expected amount of arguments
		//        if (arguments.size() != 1)
		//       {
		//          std::cerr << "[TaskTakePicture/startTask]: Wrong number of arguments. Expected: "
		//                   << numberOfArguments  << " but got: " << arguments.size() << '\n';

		//      }

		// Try to cast the arguments
		//        try
		//        {
		//            arg0 = boost::any_cast<int>(arguments[0]);
		//            arg1 = boost::any_cast<std::string>(arguments[1]);
		//            arg2 = boost::any_cast<double>(arguments[2]);
		//
		//            // Print them out
		//            std::cout << arg0 << '\n';
		//            std::cout << arg1 << '\n';
		//            std::cout << arg2 << '\n';
		//
		//            return true;
		//        }
		//        catch (boost::bad_any_cast &e)
		//        {
		//            std::cerr << "[TaskTakePicture::startTask]: " << e.what() << '\n';
		//            return false;
		//        }

        // Name of the method, used for making debugging a bit simpler
        std::string prefix = "TaskTakePicture::startTask()";
		std::cout << prefix << "Running task with " << arguments.size() << " arguments." << std::endl;

		try 
		{
			// Start the camera with our custom launch file
			std::string camera_topic = smi_.startSensor("camera", "task_take_picture", "camera1.launch");
			std::cout << prefix << "Got camera on topic '" << camera_topic << "'" << std::endl; 
			
			// Show the image in rviz
			omi_.showInRviz("image", camera_topic);
		}
		catch(error::ErrorStackUtil& e)
		{
			e.forward(prefix);
			error_handler_.append(e);
		}
	}


	std::vector<boost::any> getSolution( int subtaskNr )
	{
		// Construct an empty vector
		std::vector<boost::any> solutionVector;

		// Check the subtask number
//		if ( subtaskNr == 0)
//		{
//			boost::any retArg0 = arg0;
//			boost::any retArg1 = arg1;
//			boost::any retArg2 = arg2;
//
//			solutionVector.push_back(retArg0);
//			solutionVector.push_back(retArg1);
//			solutionVector.push_back(retArg2);
//		}

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

	// Create interfaces for accessing temoto devices
	OutputManagerInterface omi_;
	SensorManagerInterface smi_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskTakePicture, Task);
