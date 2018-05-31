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
class TaShowCamera: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaShowCamera()
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

  TEMOTO_INFO(" TaShowCamera: Showing '%s' in '%s' @ '%s' topic", what_0_word_in.c_str(),
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

std::vector<TTP::Subject> getSolution()
{
  // Construct an empty vector
  std::vector<TTP::Subject> return_subjects;

  return return_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

~TaShowCamera()
{
  TEMOTO_INFO ("Action implementation destructed");
}

private:

// Create sensor manager interface object for accessing sensor manager
output_manager::OutputManagerInterface<TaShowCamera> omi_;

// Nodehandle for subscribers and publishers
ros::NodeHandle nh_;


};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaShowCamera, TTP::BaseTask);
