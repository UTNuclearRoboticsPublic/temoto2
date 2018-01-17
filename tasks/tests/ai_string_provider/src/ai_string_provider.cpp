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
class StringProvider: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

StringProvider()
{
  // Do something here if needed
  TEMOTO_INFO("Object constructed");
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
  std::string what_0_word_out;
  std::string what_0_data_0_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  TEMOTO_INFO_STREAM("The word I received: " << what_0_word_in);
  what_0_data_0_out = "/magnificent_topic_data/over_here";
  what_0_word_out = what_0_word_in + " " + what_0_word_in;

// -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}



~StringProvider()
{
    TEMOTO_INFO("Object destructed");
}

private:

std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(StringProvider, TTP::BaseTask);
