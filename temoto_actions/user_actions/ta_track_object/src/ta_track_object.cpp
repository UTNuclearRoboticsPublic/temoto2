
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://utnuclearroboticspublic.github.io/temoto2
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include <visualization_msgs/Marker.h>

/* 
 * ACTION IMPLEMENTATION of TaTrackObject 
 */
class TaTrackObject : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaTrackObject()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaTrackObject constructed");
}
    
/* REQUIRED BY TEMOTO */
void startTask(TTP::TaskInterface task_interface)
{
  input_subjects = task_interface.input_subjects_;
  switch (task_interface.id_)
  {
        
    // Interface 0
    case 0:
      startInterface_0();
      break;

  }
}

/* REQUIRED BY TEMOTO */
std::vector<TTP::Subject> getSolution()
{
  return output_subjects;
}

~TaTrackObject()
{
  TEMOTO_INFO("TaTrackObject destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

// Nodehandle
ros::NodeHandle n_;

// Create context manager interface object for context manager manager
context_manager::ContextManagerInterface <TaTrackObject> cmi_;

    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  /* DECLARATION OF OUTPUT SUBJECTS */
  std::string  what_0_word_out;
  std::string  what_0_data_0_out;


  // Initialize the sensor manager interface
  cmi_.initialize(this);

  // Start tracking the object and return the topic to the object
  what_0_data_0_out = cmi_.trackObject(what_0_word_in);
  what_0_word_out = "object container";

  
  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));

  output_subjects.push_back(what_0_out);

}

}; // TaTrackObject class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackObject, TTP::BaseTask);
