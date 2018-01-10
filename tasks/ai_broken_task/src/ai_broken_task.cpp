/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                       // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "algorithm_manager/algorithm_manager_interface.h"

// First implementaton
class BrokenTask: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

BrokenTask()
{
  // Do something here if needed
  TASK_INFO("BrokenTask constructed");
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

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------
  try
  {
    TASK_INFO_STREAM("Starting the broken task");
    TASK_INFO_STREAM("Starting the timer ...");
    timer = nh_.createTimer(ros::Duration(1), &BrokenTask::timerCallback, this);

    TASK_INFO_STREAM("Timer started, you have " << timer_counter << " seconds until the AImp crashes");

  }
  catch( error::ErrorStack& error_stack )
  {
    SEND_ERROR(error_stack);
  }

// -------------------------------</ USER CODE >-------------------------------

}

/*
 * Timer callback
 */
void timerCallback(const ros::TimerEvent&)
{
  TASK_INFO_STREAM(--timer_counter << " seconds");

  // If the timer reaches 0, throw an exception
  if (timer_counter <= 0)
  {
    TASK_INFO_STREAM("Timer reached to 0. Throwing ...");
    std::cout << test_vector.at(2);
    // throw;
  }
}

/*
 * Destructor
 */
~BrokenTask()
{
    TASK_INFO("BrokenTask destructed");
}

private:

  ros::NodeHandle nh_;

  ros::Timer timer;

  int timer_counter = 3;

  std::vector<int> test_vector;

  std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(BrokenTask, TTP::BaseTask);
