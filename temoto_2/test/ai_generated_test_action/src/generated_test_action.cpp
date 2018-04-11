
/*##############################################################
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
 ##############################################################*/

/* REQUIRED BY TEMOTO */
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes


// ---> YOUR HEADER FILES HERE <--- //


/* 
 * ACTION IMPLEMENTATION of TestAI 
 */
class TestAI : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TestAI()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TestAI constructed");
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
    
    // Interface 1
    case 1:
      startInterface_1();
      break;

  }
}

/* REQUIRED BY TEMOTO */
std::vector<TTP::Subject> getSolution()
{
  return output_subjects;
}

~$(arg ai_class_name)()
{
  TEMOTO_INFO("$(arg ai_class_name) destructed");
}

//#################### END OF REQUIRED PUBLIC INTERFACE ####################//


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

  TTP::Subject where_1_in = TTP::getSubjectByType("where", input_subjects);
  std::string  where_1_word_in = where_1_in.words_[0];
  std::string  where_1_data_0_in = boost::any_cast<std::string>(where_1_in.data_[0].value);
  float        where_1_data_1_in = boost::any_cast<float>(where_1_in.data_[1].value);

  /* DECLARATION OF OUTPUT SUBJECTS */
  std::string  what_0_word_out;

  std::string  where_1_word_out;
  std::string  where_1_data_0_out;
  float        where_1_data_1_out;


  /* * * * * * * * * * * * * * * 
   *                           *
   * ===> YOUR CODE HERE <===  *
   *                           *
   * * * * * * * * * * * * * * */

  
  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();

  output_subjects.push_back(what_0_out);

  TTP::Subject where_1_out("where", where_1_word_out);
  where_1_out.markComplete();
  where_1_out.data_.emplace_back("topic", boost::any_cast<std::string>(where_1_data_0_out));
  where_1_out.data_.emplace_back("number", boost::any_cast<float>(where_1_data_1_out));

  output_subjects.push_back(where_1_out);

}
    
/*
 * Interface 1 body
 */
void startInterface_1()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  TTP::Subject where_1_in = TTP::getSubjectByType("where", input_subjects);
  std::string  where_1_word_in = where_1_in.words_[0];
  std::string  where_1_data_0_in = boost::any_cast<std::string>(where_1_in.data_[0].value);
  float        where_1_data_1_in = boost::any_cast<float>(where_1_in.data_[1].value);


  /* * * * * * * * * * * * * * * 
   *                           *
   * ===> YOUR CODE HERE <===  *
   *                           *
   * * * * * * * * * * * * * * */

  
}

};

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TestAI, TTP::BaseTask);
