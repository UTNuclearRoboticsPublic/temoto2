
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

// Task specific includes
#include "ros/ros.h"
#include "human_msgs/Hands.h"
#include "context_manager/context_manager_containers.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* 
 * ACTION IMPLEMENTATION of TaTrackHand 
 */
class TaTrackHand : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaTrackHand()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaTrackHand constructed");
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

~TaTrackHand()
{
  TEMOTO_INFO("TaTrackHand destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

ros::NodeHandle nh_;
ros::Subscriber hand_subscriber_;
ros::Publisher tracked_object_publisher_;
context_manager::ObjectPtr tracked_object_;

void handDataCb(human_msgs::Hands msg)
{
  // Use tf2 to transform hand pose from its optical frame to the camera_link
  geometry_msgs::Point pt = msg.right_hand.palm_pose.pose.position;
  tf2::Vector3 vec_orig(pt.x, pt.y, pt.z);
  geometry_msgs::Quaternion q_msg = msg.right_hand.palm_pose.pose.orientation;
  tf2::Quaternion q_orig(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Transform transform(q_orig, vec_orig);

  tf2::Quaternion q2;
  q2.setRPY(3.14, 0, 1.57);
  tf2::Quaternion q3;
  q3.setRPY(-1.57, -1.57, 0);
  tf2::Transform transform2(q2, tf2::Vector3(0, -0.05, 0.3)); // translate hand to the center of the workspace
  tf2::Vector3 vec = transform2*vec_orig;
  tf2::Quaternion q = q2*q_orig*q3;

  geometry_msgs::Point position2;
  position2.x = vec.x();
  position2.y = vec.y();
  position2.z = vec.z();
  tracked_object_->pose.pose.position = position2;

  geometry_msgs::Quaternion q_msg2;
  q_msg2.x = q.x();
  q_msg2.y = q.y();
  q_msg2.z = q.z();
  q_msg2.w = q.w();
  tracked_object_->pose.pose.orientation = q_msg2;
  tracked_object_->pose.header = msg.right_hand.palm_pose.header;
  tracked_object_->pose.header.frame_id = "world";

  // Publish the tracked object
  tracked_object_publisher_.publish(*tracked_object_);

  std::cout << "Detected right hand jeboi" << std::endl;
}
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  TTP::Subject what_1_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  std::string  what_1_data_1_in = boost::any_cast<std::string>(what_1_in.data_[1].value);
  context_manager::ObjectPtr  what_1_data_2_in = boost::any_cast<context_manager::ObjectPtr>(what_1_in.data_[2].value);

  try
  {
    TEMOTO_INFO_STREAM("Starting to track object: '" << what_1_data_2_in->name << "'"
                       << " with tag_id = " << what_1_data_2_in->tag_id);
    TEMOTO_INFO_STREAM("The tracker type is: '" << what_1_data_2_in->detection_methods[0] << "'");
    TEMOTO_INFO_STREAM("Receiving hand data from topic: '" << what_1_data_0_in << "'");
    TEMOTO_INFO_STREAM("Publishing the tracked object to topic: '" << what_1_data_1_in << "'");

    // Subscribe to the AR tag data topic
    hand_subscriber_ = nh_.subscribe(what_1_data_0_in, 10, &TaTrackHand::handDataCb, this);

    // Advertise the tracked object topic
    tracked_object_publisher_ = nh_.advertise<temoto_2::ObjectContainer>(what_1_data_1_in, 10);

    // Assign the object pointer to the local object pointer "tracked_object"
    tracked_object_ = what_1_data_2_in;

    TEMOTO_INFO_STREAM("Subscribed to hand data topic: " << what_1_data_0_in);

  }
  catch( temoto_core::error::ErrorStack& error_stack )
  {
    SEND_ERROR(error_stack);
  }  
}

}; // TaTrackHand class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackHand, TTP::BaseTask);
