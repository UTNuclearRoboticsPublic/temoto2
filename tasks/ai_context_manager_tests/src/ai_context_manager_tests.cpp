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

// First implementaton
class ContextManagerTests: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

ContextManagerTests()
{
  // Do something here if needed
  TASK_INFO("ContextManagerTests constructed");
}

// startTask with arguments
bool startTask(TTP::TaskInterface task_interface)
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

      // Interface 1
      case 1:
          startInterface_1();
      break;
  }

  return true;
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

  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

  try
  {
    // Initialize the sensor manager interface
    cmi_.initialize(this);

    // Create a cylinder object
    temoto_2::ObjectContainer cylinder;
    cylinder.name = "small cylinder";
    cylinder.detection_methods.push_back(temoto_2::ObjectContainer::ARTAG);
    cylinder.tag_id = 2;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
    primitive.dimensions.push_back(0.2);  // Height
    primitive.dimensions.push_back(0.06); // Radius
    cylinder.primitive = primitive;

    // Add a new object to the context manager
    cmi_.addWorldObjects(cylinder);

    // Pass the name of the object to the output
    what_0_word_out = cylinder.name;
  }
  catch( error::ErrorStack& e )
  {
    error_handler_.forwardAndAppend(e, prefix);
  }

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

  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

  try
  {
    // Initialize the sensor manager interface
    cmi_.initialize(this);

    // Start a tracker
    TopicContainer tracker_topics = cmi_.startTracker("tests");

    // Pass the name of the object to the output
    what_0_word_out = tracker_topics.getOutputTopic("data");

    std::cout << "GOT TOPIC: " << what_0_word_out << std::endl;
  }
  catch( error::ErrorStack& e )
  {
    error_handler_.forwardAndAppend(e, prefix);
  }

// -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
}

~ContextManagerTests()
{
    TASK_INFO("ContextManagerTests destructed");
}

private:

// Nodehandle
ros::NodeHandle n_;

// Create context manager interface object for context manager manager
ContextManagerInterface <ContextManagerTests> cmi_;

std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(ContextManagerTests, TTP::BaseTask);
