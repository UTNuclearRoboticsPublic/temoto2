
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


// ---> YOUR HEADER FILES HERE <--- //


/* 
 * ACTION IMPLEMENTATION of TaFirstTest 
 */
class TaFirstTest : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaFirstTest()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaFirstTest constructed");
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

~TaFirstTest()
{
  TEMOTO_INFO("TaFirstTest destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

/* * * * * * * * * * * * * * * * * * 
 *                                 *
 * ===> YOUR CUSTOM VARIABLES <=== *
 *       AND FUNCTIONS HERE        *
 *                                 *
 * * * * * * * * * * * * * * * * * */

    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];


  while (!stop_task_)
  {
    TEMOTO_INFO("First test reporting");
    ros::Duration(1.5).sleep();
  }

  
}

}; // TaFirstTest class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFirstTest, TTP::BaseTask);
