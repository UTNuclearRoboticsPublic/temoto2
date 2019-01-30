
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
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "temoto_context_manager/context_manager_containers.h"

/* 
 * ACTION IMPLEMENTATION of TaTrackArtag 
 */
class TaTrackArtag : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaTrackArtag()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaTrackArtag constructed");
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

~TaTrackArtag()
{
  TEMOTO_INFO("TaTrackArtag destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

ros::NodeHandle nh_;
ros::Subscriber artag_subscriber_;
ros::Publisher tracked_object_publisher_;
temoto_context_manager::ObjectPtr tracked_object_;
uint32_t tag_id_;

void artagDataCb(ar_track_alvar_msgs::AlvarMarkers msg)
{

  // Look for the marker with the required tag id
  for (auto& artag : msg.markers)
  {
    if (artag.id == tag_id_)
    {
      TEMOTO_INFO_STREAM( "AR tag with id = " << tag_id_ << " found");

      // Update the pose of the object
      tracked_object_->pose.pose = artag.pose.pose;
      tracked_object_->pose.header = artag.header;

      // Publish the tracked object
      tracked_object_publisher_.publish(*tracked_object_);

      // TODO: do something reasonable if multiple markers with the same tag id are present
    }
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

  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  std::string  what_1_data_1_in = boost::any_cast<std::string>(what_1_in.data_[1].value);
  temoto_context_manager::ObjectPtr  what_1_data_2_in = boost::any_cast<temoto_context_manager::ObjectPtr>(what_1_in.data_[2].value);


  try
  {
    TEMOTO_INFO_STREAM("Starting to track object: '" << what_1_data_2_in->name << "'"
                       << " with tag_id = " << what_1_data_2_in->tag_id);
    TEMOTO_INFO_STREAM("The tracker type is: '" << what_1_data_2_in->detection_methods[0] << "'");
    TEMOTO_INFO_STREAM("Receiving AR-Tags from topic: '" << what_1_data_0_in << "'");
    TEMOTO_INFO_STREAM("Publishing the tracked object to topic: '" << what_1_data_1_in << "'");

    // Get the tag id
    tag_id_ = what_1_data_2_in->tag_id;

    // Subscribe to the AR tag data topic
    artag_subscriber_ = nh_.subscribe(what_1_data_0_in, 10, &TaTrackArtag::artagDataCb, this);

    // Advertise the tracked object topic
    tracked_object_publisher_ = nh_.advertise<temoto_context_manager::ObjectContainer>(what_1_data_1_in, 10);

    // Assign the object pointer to the local object pointer "tracked_object"
    tracked_object_ = what_1_data_2_in;

    TEMOTO_INFO_STREAM("Subscribed to AR-Tag data topic: " << what_1_data_0_in);

  }
  catch( temoto_core::error::ErrorStack& error_stack )
  {
    SEND_ERROR(error_stack);
  }
}

}; // TaTrackArtag class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackArtag, temoto_nlp::BaseTask);
