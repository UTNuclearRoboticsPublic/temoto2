/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

// First implementaton
class ContextManagerTests1: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

ContextManagerTests1()
{
  // Do something here if needed
  TEMOTO_INFO("Action implementation constructed");
}

// startTask with arguments
void startTask(TTP::TaskInterface task_interface)
{
  // TODO: This is a hack for Veiko
  task_alias = task_interface.alias_;

  // < AUTO-GENERATED, DO NOT MODIFY >
  input_subjects = task_interface.input_subjects_;
  switch(task_interface.id_)
  {
    // Interface 0
    case 0:
      startInterface_0();
    break;

    // Interface 0
    case 1:
      startInterface_1();
    break;
  }
  // </ AUTO-GENERATED, DO NOT MODIFY >
}

std::string getStatus()
{
  std::string str = "healthy";
  return str;
}

std::vector<TTP::Subject> getSolution()
{
  return output_subjects;
}


/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * Interface 0 body
 */
void startInterface_0()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  // Creating output variables
  std::string  what_0_word_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  // Initialize the sensor manager interface
  cmi_.initialize(this);

  // Create a cylinder object
  temoto_2::ObjectContainer cylinder;
  cylinder.name = "cylinder";

  TEMOTO_INFO_STREAM("The tracking method for the cylinder is:" << temoto_2::ObjectContainer::ARTAG);

  cylinder.detection_methods.push_back(temoto_2::ObjectContainer::ARTAG);
  //cylinder.tag_id = 4;
  cylinder.tag_id = 9;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.02;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  cylinder.marker = marker;


  tf2::Quaternion q;
  q.setRPY(-1.57, 1.57, 1.57);
  cylinder.obj_relative_pose.position.x = -0.2;
  cylinder.obj_relative_pose.orientation.x = q.x();
  cylinder.obj_relative_pose.orientation.y = q.y();
  cylinder.obj_relative_pose.orientation.z = q.z();
  cylinder.obj_relative_pose.orientation.w = q.w();

//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
//  primitive.dimensions.push_back(0.2);   // Height
//  primitive.dimensions.push_back(0.06);  // Radius
//  cylinder.primitive = primitive;

  // Add a new object to the context manager
  cmi_.addWorldObjects(cylinder);

  // Pass the name of the object to the output
  what_0_word_out = cylinder.name;

  // -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

/*
 * Interface 1 body
 */
void startInterface_1()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  // Creating output variables
  std::string  what_0_word_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  // Initialize the sensor manager interface
  cmi_.initialize(this);

  // Create a hand object
  temoto_2::ObjectContainer hand;
  hand.name = "right hand";

  TEMOTO_INFO_STREAM("The tracking method for the hand is:" << temoto_2::ObjectContainer::HAND);

  hand.detection_methods.push_back(temoto_2::ObjectContainer::HAND);

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.02;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  hand.marker = marker;

  // Add a new object to the context manager
  cmi_.addWorldObjects(hand);

  // Pass the name of the object to the output
  what_0_word_out = hand.name;

  // -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

~ContextManagerTests1()
{
    TEMOTO_INFO("Action implementation destructed");
}

private:

// Nodehandle
ros::NodeHandle n_;

// Create context manager interface object for context manager manager
context_manager::ContextManagerInterface <ContextManagerTests1> cmi_;

std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(ContextManagerTests1, TTP::BaseTask);
