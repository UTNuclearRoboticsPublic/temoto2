// Things that have to be included
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// First implementaton
class ShowEnv : public TTP::BaseTask
{
public:
  
/* REQUIRED BY TEMOTO */
ShowEnv()
{
  /*
   * Do something here if needed
   */
  TEMOTO_INFO("ShowEnv constructed");
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

/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* < AUTO-GENERATED, DO NOT MODIFY > */

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string what_0_word_in = what_0_in.words_[0];

  /* </ AUTO-GENERATED, DO NOT MODIFY > */

  // --------------------------------< USER CODE >-------------------------------

  

  // --------------------------------</ USER CODE >-------------------------------
}

~ShowEnv()
{
  TEMOTO_INFO("ShowEnv destructed");
}

private:

  /*
   * Do some private stuff if needed
   */

};

/* Required by class loader */
CLASS_LOADER_REGISTER_CLASS(ShowEnv, TTP::BaseTask);

