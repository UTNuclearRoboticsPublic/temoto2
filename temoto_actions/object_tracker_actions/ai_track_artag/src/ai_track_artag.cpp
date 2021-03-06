/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	AImp that tracks an artag
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"         // The base task
#include <class_loader/class_loader.h>       // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "context_manager/context_manager_containers.h"

// First implementaton
class TrackArtag: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TrackArtag()
{
    // Do something here if needed
    TASK_INFO("TrackArtag constructed");
}

// startTask with arguments
void startTask(TTP::TaskInterface task_interface)
{
// * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;
    switch(task_interface.id_)
    {
        // Interface 0
        case 0:
            startInterface_0();
        break;
    }
// * AUTO-GENERATED, DO NOT MODIFY *
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  TTP::Subject what_1_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  std::string  what_1_data_1_in = boost::any_cast<std::string>(what_1_in.data_[1].value);
  context_manager::ObjectPtr  what_1_data_2_in = boost::any_cast<context_manager::ObjectPtr>(what_1_in.data_[2].value);

  // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

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
    artag_subscriber_ = nh_.subscribe(what_1_data_0_in, 10, &TrackArtag::artagDataCb, this);

    // Advertise the tracked object topic
    tracked_object_publisher_ = nh_.advertise<temoto_2::ObjectContainer>(what_1_data_1_in, 10);

    // Assign the object pointer to the local object pointer "tracked_object"
    tracked_object_ = what_1_data_2_in;

    TEMOTO_INFO_STREAM("Subscribed to AR-Tag data topic: " << what_1_data_0_in);

  }
  catch( error::ErrorStack& error_stack )
  {
    SEND_ERROR(error_stack);
  }

// -------------------------------</ USER CODE >-------------------------------

}

std::string getStatus()
{
    std::string str = "healthy";
    return str;
}

std::vector<TTP::Subject> getSolution()
{
    return output_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

void artagDataCb(ar_track_alvar_msgs::AlvarMarkers msg)
{

  // Look for the marker with the required tag id
  for (auto& artag : msg.markers)
  {
    if (artag.id == tag_id_)
    {
      TEMOTO_DEBUG_STREAM( "AR tag with id = " << tag_id_ << " found");

      // Update the pose of the object
      tracked_object_->pose.pose = artag.pose.pose;
      tracked_object_->pose.header = artag.header;

      // Publish the tracked object
      tracked_object_publisher_.publish(*tracked_object_);

      // TODO: do something reasonable if multiple markers with the same tag id are present
    }
  }
}

~TrackArtag()
{
  TASK_INFO("TrackArtag destructed");
}

private:

ros::NodeHandle nh_;
ros::Subscriber artag_subscriber_;
ros::Publisher tracked_object_publisher_;
context_manager::ObjectPtr tracked_object_;
uint32_t tag_id_;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TrackArtag, TTP::BaseTask);
