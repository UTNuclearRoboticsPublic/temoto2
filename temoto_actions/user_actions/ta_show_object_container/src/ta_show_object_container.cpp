
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

// Action specific includes
#include "ros/ros.h"
#include "output_manager/output_manager_interface.h"
#include "temoto_context_manager/ObjectContainer.h"
#include <visualization_msgs/Marker.h>
#include "human_msgs/Hands.h"



/* 
 * ACTION IMPLEMENTATION of TaShowObjectContainer 
 */
class TaShowObjectContainer : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaShowObjectContainer()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaShowObjectContainer constructed");
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

~TaShowObjectContainer()
{
  TEMOTO_INFO("TaShowObjectContainer destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

// Create sensor manager interface object for accessing sensor manager
output_manager::OutputManagerInterface<TaShowObjectContainer> omi_;

// Nodehandle for subscribers and publishers
ros::NodeHandle nh_;

// Subscriber to temoto objects
ros::Subscriber object_subscriber_;

// Marker publisher
ros::Publisher marker_publisher_;


/**
 * @brief objectContainerCb
 * @param msg
 */
void objectContainerCb(temoto_context_manager::ObjectContainer msg)
{
  //TEMOTO_DEBUG_STREAM(" Publishing the marker");

  /*
   * The ObjectContainer already contains the marker message, but only
   * its visual parameters are initialized.
   */
  msg.marker.header = msg.pose.header;
  msg.marker.pose = msg.pose.pose;
  msg.marker.ns = msg.name;
  msg.marker.id = 0;
  msg.marker.lifetime = ros::Duration();

  if (marker_publisher_)
  {
    marker_publisher_.publish(msg.marker);
    std::cout << "publishing a marker message" << std::endl;
  }
  else
  {
    TASK_ERROR("Excepition ERROR kutsuge 112 cobra");
  }
}
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);

  temoto_nlp::Subject where_1_in = temoto_nlp::getSubjectByType("where", input_subjects);
  std::string  where_1_word_in = where_1_in.words_[0];


  // Initialize the output manager interface
  omi_.initialize(this);

  std::string marker_topic = temoto_core::common::getAbsolutePath(output_manager::generic_topics::MARKER);

  // Subscribe to the object container topic
  object_subscriber_ = nh_.subscribe(what_0_data_0_in, 10, &TaShowObjectContainer::objectContainerCb, this);

  // Advertise the marker topic
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

  TEMOTO_INFO("Receiving object container data on topic: '%s'", what_0_data_0_in.c_str());
  TEMOTO_INFO("Showing '%s' in '%s' @ '%s' topic", what_0_word_in.c_str(),
            where_1_word_in.c_str(), marker_topic.c_str());

  // Show the marker in rviz
  omi_.showInRviz("marker", marker_topic);  
}

}; // TaShowObjectContainer class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaShowObjectContainer, temoto_nlp::BaseTask);
