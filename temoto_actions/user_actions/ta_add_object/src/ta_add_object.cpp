
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

#include "ros/ros.h"
#include "temoto_context_manager/context_manager_interface.h"
#include <visualization_msgs/Marker.h>

/* 
 * ACTION IMPLEMENTATION of TaAddObject 
 */
class TaAddObject : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaAddObject()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaAddObject constructed");
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

~TaAddObject()
{
  TEMOTO_INFO("TaAddObject destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

  // Nodehandle
  ros::NodeHandle n_;

  // Create context manager interface object for context manager manager
  temoto_context_manager::ContextManagerInterface <TaAddObject> cmi_;

    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];


  // Initialize the sensor manager interface
  cmi_.initialize(this);

  // Create a hand object
  temoto_context_manager::ObjectContainer hand;
  hand.name = "right hand";

  //TEMOTO_INFO_STREAM("The tracking method for the hand is:" << temoto_context_manager::ObjectContainer::HAND);
  TEMOTO_INFO_STREAM("The tracking method for the hand is:" << temoto_context_manager::ObjectContainer::ARTAG);

  hand.detection_methods.push_back(temoto_context_manager::ObjectContainer::HAND);
  hand.detection_methods.push_back(temoto_context_manager::ObjectContainer::ARTAG);
  hand.tag_id = 2;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.02;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  hand.marker = marker;

  // Add a new object to the context manager
  cmi_.addWorldObjects(hand);

}

}; // TaAddObject class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaAddObject, temoto_nlp::BaseTask);
