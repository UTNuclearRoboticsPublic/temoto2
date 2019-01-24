
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
#include "context_manager/context_manager_interface.h"

/* 
 * ACTION IMPLEMENTATION of TaGetNumber 
 */
class TaGetNumber : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaGetNumber()
{
  TEMOTO_INFO("TaGetNumber constructed");
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

~TaGetNumber()
{
  TEMOTO_INFO("TaGetNumber destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

// Context Manager Interface object
context_manager::ContextManagerInterface<TaGetNumber> cmi_;

/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  // Initialize context manager interface
  cmi_.initialize(this);

  // Get some numbers
  TEMOTO_INFO("asking for some numbers");

  int n0_0 = cmi_.getNumber(1);
  int n1_0 = cmi_.getNumber(22);
  int n2_0 = cmi_.getNumber(333);

  TEMOTO_INFO("got %d, %d, %d", n0_0, n1_0, n2_0);

  // Get some more numbers
  TEMOTO_INFO("asking the same numbers again");

  int n0_1 = cmi_.getNumber(n0_0);
  int n1_1 = cmi_.getNumber(n1_0);
  int n2_1 = cmi_.getNumber(n2_0);

  TEMOTO_INFO("got %d, %d, %d", n0_1, n1_1, n2_1);
}

}; // TaGetNumber class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaGetNumber, temoto_nlp::BaseTask);
