/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that loads the robot using robot manager interface.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"             // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "output_manager/output_manager_interface.h"
#include "robot_manager/robot_manager_interface.h"

class TaskRobot: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskRobot() : rmi_(this)
    {
        // Do something here if needed
        TASK_INFO("Task Robot constructed");
    }

    // startTask without arguments
    bool startTask()
    {

        return true;
    }

    // startTask with arguments
	bool startTask(int subtaskNr, std::vector<boost::any> arguments )
	{
    std::string prefix = "TaskRobot::startTask()";
    std::cout << prefix << "Running task with " << arguments.size() << " arguments." << std::endl;

    // initialize interfaces
    omi_.initialize(this);
    rmi_.initialize();
    
    if (arguments.size() == 1 && boost::any_cast<std::string>(arguments[0]) == "plan")
    {
      TASK_DEBUG("%s STARTING PLANNING", prefix.c_str());
      geometry_msgs::Pose target_pose;
      rmi_.plan(target_pose);
      TASK_DEBUG("%s FINISHED PLANNING", prefix.c_str());
    }
    else if (arguments.size() == 1 && boost::any_cast<std::string>(arguments[0]) == "execute")
    {
      TASK_DEBUG("%s STARTING EXECUTING", prefix.c_str());
      geometry_msgs::Pose pose;
      rmi_.execute();
      TASK_DEBUG("%s FINISHED EXECUTING", prefix.c_str());
    }
    else
    {
      try
      {
        // Load the ur5 robot
        rmi_.loadRobot("ur5");
        std::string rviz_conf = rmi_.getMoveitRvizConfig();
        omi_.showInRviz("robot", "", rviz_conf);

        // DO NOT EXIT THIS FUNCTION
        ros::waitForShutdown();

      }
      catch (error::ErrorStackUtil& e)
      {
        e.forward(prefix);
        error_handler_.append(e);
      }
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


	~TaskRobot()
	{
		TASK_INFO("[TaskRobot::~TaskRobot] TaskRobot destructed");
	}

private:

	// Create interfaces for accessing temoto devices
	OutputManagerInterface omi_;
	RobotManagerInterface<TaskRobot> rmi_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskRobot, Task);
