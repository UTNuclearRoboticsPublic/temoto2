
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
#include "temoto_nlp/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes
#include "process_manager/external_resource_manager_interface.h"

/* 
 * ACTION IMPLEMENTATION of TaControlTurtlebot 
 */
class TaControlTurtlebot : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaControlTurtlebot()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaControlTurtlebot constructed");
}
    
/* REQUIRED BY TEMOTO */
void startTask(temoto_nlp::TaskInterface task_interface)
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
std::vector<temoto_nlp::Subject> getSolution()
{
  return output_subjects;
}

~TaControlTurtlebot()
{
  TEMOTO_INFO("TaControlTurtlebot destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

external_resource_manager::ExternalResourceManagerInterface<TaControlTurtlebot> ermi_;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  temoto_nlp::Subject where_1_in = temoto_nlp::getSubjectByType("where", input_subjects);
  std::string  where_1_word_in = where_1_in.words_[0];

  // Initialize the external resource manager interface
  ermi_.initialize(this);

  // Start a node
  ermi_.loadResource("rviz", "rviz");

  TEMOTO_INFO_STREAM("I am controlling the turtlebot");
  
}

}; // TaControlTurtlebot class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaControlTurtlebot, temoto_nlp::BaseTask);
