/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *              * Replicates the functionalities of Temoto 1.0
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "context_manager/human_context/human_context_interface.h"
#include "output_manager/output_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"
#include <visualization_msgs/Marker.h>

// First implementaton
class TaskTemoto: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

TaskTemoto()
{
    // Do something here if needed
   // ROS_INFO("TaskTemoto constructed");
}
/*
 * startTask without arguments
 */
bool startTask()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    // Build a speech specifier
    // TODO: This shoud be done via speech specifier helper class
    /*
    std::vector <temoto_2::speechSpecifier> speechSpecifiers;
    temoto_2::speechSpecifier speechSpecifier;
    speechSpecifier.dev = "device";
    speechSpecifier.type = "text";
    speechSpecifiers.push_back(speechSpecifier);
    */

    hci_.initialize(this);
    omi_.initialize(this);
    smi_.initialize(this);

    // Build a gesture specifier
    // TODO: This shoud be done via speech specifier helper class
    std::vector <temoto_2::GestureSpecifier> gesture_specifiers;
    temoto_2::GestureSpecifier gesture_specifier;
    gesture_specifier.dev = "device";
    gesture_specifier.type = "hand";
    gesture_specifiers.push_back(gesture_specifier);


    // Register the speech request and bind a callback
    /*
    if ( !hci_.getSpeech(speechSpecifiers, &TaskTemoto::speechCallback, this ) )
    {
        ROS_ERROR("[TaskTemoto::startTask()]: getSpeech request failed");
        return 1;
    }
    */


    try
    {
        // Advertise the marker topic
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("/temoto_task_markers", 1);

        // Register the gesture request and bind a callback
        hci_.getGestures(gesture_specifiers, &TaskTemoto::gestureCallback, this );

        // Make rviz load a marker display
        omi_.showInRviz( "marker", "/temoto_task_markers" );

        // Pointless dummy sensor manager request
        smi_.startSensor( "hand", "temoto_2", "non_existing_file.launch" );

    }
    catch( error::ErrorStackUtil& e )
    {
        e.forward( prefix );
        error_handler_.append(e);
    }


    while (!stop_task_)
    {
        std::cout << "in da loop" << std::endl;
        ros::Duration(1).sleep();
    }

    return true;
}

/*
 *  startTask with arguments
 */
bool startTask(int subtaskNr, std::vector<boost::any> arguments )
{
    // Check if arguments vector contains expected amount of arguments
    if (arguments.size() != numberOfArguments)
    {
        std::cerr << "[TaskTemoto::startTask]: Wrong number of arguments. Expected: "
                  << numberOfArguments  << " but got: " << arguments.size() << '\n';

        return false;
    }

    // If it does, try to cast the arguments
    try
    {
        arg_0 = boost::any_cast<std::string>(arguments[0]);
        print_ = false;
        startTask();
        return true;

    }
    catch (boost::bad_any_cast &e)
    {
        std::cerr << "[TaskStop::startTask]: " << e.what() << '\n';
        return false;
    }
}

/*
 *  Get the status of the task
 */

std::string getStatus()
{
    std::string str = "healthy";
    return str;
}

/*
 *  Get a solution vector
 */

std::vector<boost::any> getSolution( int subtaskNr )
{
    // Construct an empty vector
    std::vector<boost::any> solutionVector;

    return solutionVector;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

// Callback for processing speech
void speechCallback(std_msgs::String msg)
{
    TASK_INFO("Speech callback got: %s", msg.data.c_str());
}

// Callback for processing gestures
void gestureCallback( human_msgs::Hands gesture )
{ 
    TASK_INFO("wadup");

    std::cout << __FUNCTION__ << std::endl;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "hand_indicator";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose = gesture.left_hand.palm_pose.pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    /*
    while (marker_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_INFO("Please create a subscriber to the marker");
      sleep(1);
    }
    */
    if(marker_pub_)
    {
        marker_pub_.publish(marker);
    }
    else
    {
        TASK_ERROR("Excepition ERROR kutsuge 112 cobra");
    }
}

~TaskTemoto()
{
    TASK_INFO("[TaskTemoto::~TaskTemoto]TaskTemoto destructed");
}

private:

/**
 * @brief hci_
 */
HumanContextInterface <TaskTemoto> hci_;

/**
 * @brief omi_
 */
OutputManagerInterface omi_;

/**
 * @brief smi_
 */
SensorManagerInterface smi_;

/**
 * @brief class_name_
 */
std::string class_name_ = "TaskTemoto";

/**
 * @brief n_
 */
ros::NodeHandle n_;

/**
 * @brief marker_pub_
 */
ros::Publisher marker_pub_;

bool stop_task_ = true;


// Random stuff for testing
int numberOfArguments = 1;
std::string arg_0;
bool print_ = true;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskTemoto, Task);
