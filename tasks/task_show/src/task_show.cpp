/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *		* Demonstrate dynamic subscription
 *              * Brings up a terminal that allows to send commands to the core
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "output_manager/output_manager_interface.h"
#include "temoto_2/ObjectContainer.h"
#include <visualization_msgs/Marker.h>
#include "human_msgs/Hands.h"

// First implementaton
class TaskShow: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaskShow()
{
    // Do something here if needed
    TEMOTO_INFO("Action implementation constructed");
}

// startTask with arguments
void startTask(TTP::TaskInterface task_interface)
{
  // < AUTO-GENERATED, DO NOT MODIFY >
  input_subjects = task_interface.input_subjects_;

  try
  {
    switch(task_interface.id_)
    {
      // Interface 0
      case 0:
        startInterface_0();
      break;

      // Interface 1
      case 1:
        startInterface_1();
      break;

      // Interface 2
      case 2:
        startInterface_2();
      break;
    }
  }
  catch(boost::bad_any_cast& e)
  {
      std::cout << "OH NO: " << e.what() << std::endl;
  }

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);

  TTP::Subject where_0_in = TTP::getSubjectByType("where", input_subjects);
  std::string  where_0_word_in = where_0_in.words_[0];

  // Creating output variables
  std::string  what_0_word_out;
  std::string  what_0_data_0_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

  /* --------------------------------< USER CODE >------------------------------- */

  // Initialize the output manager interface
  omi_.initialize(this);

  TEMOTO_INFO(" TaskShow: Showing '%s' in '%s' @ '%s' topic", what_0_word_in.c_str(),
            where_0_word_in.c_str(), what_0_data_0_in.c_str());

  // Show the image in rviz
  omi_.showInRviz("image", what_0_data_0_in);

  // Fill the out part of the current task-interface
  what_0_word_out = what_0_word_in;
  what_0_data_0_out = what_0_data_0_in;

  /* --------------------------------</ USER CODE >------------------------------- */

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}


/*
 * Interface 1 body:
 * Converts object container msgs to visualization marker msgs
 */
void startInterface_1()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);

  TTP::Subject where_0_in = TTP::getSubjectByType("where", input_subjects);
  std::string  where_0_word_in = where_0_in.words_[0];

  // </ AUTO-GENERATED, DO NOT MODIFY >

  /* --------------------------------< USER CODE >------------------------------- */

  // Initialize the output manager interface
  omi_.initialize(this);

  std::string marker_topic = common::getAbsolutePath(output_manager::generic_topics::MARKER);

  // Subscribe to the object container topic
  object_subscriber_ = nh_.subscribe(what_0_data_0_in, 10, &TaskShow::objectContainerCb, this);

  // Advertise the marker topic
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

  TEMOTO_INFO("Receiving object container data on topic: '%s'", what_0_data_0_in.c_str());
  TEMOTO_INFO("Showing '%s' in '%s' @ '%s' topic", what_0_word_in.c_str(),
            where_0_word_in.c_str(), marker_topic.c_str());

  // Show the marker in rviz
  omi_.showInRviz("marker", marker_topic);

  /* --------------------------------</ USER CODE >------------------------------- */
}


/*
 * Interface 2 body:
 * Converts handtracker msgs to visualization marker msgs
 */
void startInterface_2()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);

  TTP::Subject where_0_in = TTP::getSubjectByType("where", input_subjects);
  std::string  where_0_word_in = where_0_in.words_[0];

  // </ AUTO-GENERATED, DO NOT MODIFY >

  /* --------------------------------< USER CODE >------------------------------- */

  // Initialize the output manager interface
  omi_.initialize(this);

  std::string marker_topic = common::getAbsolutePath(output_manager::generic_topics::MARKER);

  // Subscribe to the handtracker topic
  object_subscriber_ = nh_.subscribe(what_0_data_0_in, 10, &TaskShow::handCb, this);

  // Advertise the marker topic
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

  TEMOTO_INFO("Receiving handtracker data on topic: '%s'", what_0_data_0_in.c_str());
  TEMOTO_INFO("Showing '%s' in '%s' @ '%s' topic", what_0_word_in.c_str(),
            where_0_word_in.c_str(), marker_topic.c_str());

  // Show the marker in rviz
  omi_.showInRviz("marker", marker_topic);

  /* --------------------------------</ USER CODE >------------------------------- */
}

std::vector<TTP::Subject> getSolution()
{
  // Construct an empty vector
  std::vector<TTP::Subject> return_subjects;

  return return_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

/**
 * @brief objectContainerCb
 * @param msg
 */
void objectContainerCb(temoto_2::ObjectContainer msg)
{
  //TEMOTO_DEBUG_STREAM(" Publishing the marker");

  /*
   * The ObjectContainer already contains the marker message, but only
   * its visual parameters are initialized.
   */
  msg.marker.header = msg.pose.header;
  msg.marker.pose = msg.pose.pose;
  msg.marker.ns = msg.name;
  msg.marker.id = 0;
  msg.marker.lifetime = ros::Duration();

  if (marker_publisher_)
  {
    marker_publisher_.publish(msg.marker);
  }
  else
  {
    TASK_ERROR("Excepition ERROR kutsuge 112 cobra");
  }
}

/**
 * @brief handCb
 * @param hand_msg
 */
void handCb(human_msgs::Hands hand_msg)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header = hand_msg.left_hand.palm_pose.header;

  marker.ns = "hand_indicator";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and
  // CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in
  // the header
  marker.pose = hand_msg.left_hand.palm_pose.pose;
  //    double z = marker.pose.position.z;
  //    marker.pose.position.z = -0.5*marker.pose.position.y;
  //    marker.pose.position.y = -5*z;
  //    marker.pose.position.x *= -1.5;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.17;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  if (marker_publisher_)
  {
    marker_publisher_.publish(marker);
  }
  else
  {
    TASK_ERROR("Excepition ERROR kutsuge 112 cobra");
  }
}

~TaskShow()
{
  TEMOTO_INFO ("Action implementation destructed");
}

private:

// Create sensor manager interface object for accessing sensor manager
output_manager::OutputManagerInterface<TaskShow> omi_;

// Nodehandle for subscribers and publishers
ros::NodeHandle nh_;

ros::Timer timer;

// Subscriber to temoto objects
ros::Subscriber object_subscriber_;

// Subscriber to temoto objects
ros::Subscriber hand_subscriber_;

// Marker publisher
ros::Publisher marker_publisher_;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskShow, TTP::BaseTask);
