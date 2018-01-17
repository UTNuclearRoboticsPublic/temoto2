/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"
#include <visualization_msgs/Marker.h>

// First implementaton
class TaskStart : public TTP::BaseTask
{
public:
  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented /START
   * * * * * * * * * * * * * * * * * * * * * * * * */

  TaskStart()
  {
    // Do something here if needed
    TEMOTO_INFO("TaskStart constructed");
  }

  // startTask with arguments
  void startTask(TTP::TaskInterface task_interface)
  {
    // TODO: This is a hack for Veiko
    task_alias = task_interface.alias_;

    // < AUTO-GENERATED, DO NOT MODIFY >
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

      // Interface 2
      case 2:
        startInterface_2();
        break;
    }
    // </ AUTO-GENERATED, DO NOT MODIFY >
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

  /*
   * Interface 0 body
   */
  void startInterface_0()
  {
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string where_0_word_out;
    float where_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskStart got: " << what_0_word_in << std::endl;
    std::cout << "  TaskStart was called with alias: " << task_alias << std::endl;

    where_0_word_out = what_0_word_in + "land";
    where_0_data_0_out = 12.345;

    // --------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject where_0_out("where", where_0_word_out);
    where_0_out.markComplete();

    where_0_out.data_.emplace_back("number", boost::any_cast<float>(where_0_data_0_out));
    output_subjects.push_back(where_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
  }

  /*
   * Interface 1 body
   */
  void startInterface_1()
  {
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string what_0_word_out;
    std::string what_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // -------------------------------< USER CODE >-------------------------------

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    // Initialize the sensor manager interface
    smi_.initialize(this);

    SensorTopicsReq requested_topics;
    requested_topics.addOutputTopicType("camera_data");
    requested_topics.addOutputTopicType("camera_info");

    TEMOTO_INFO(" Starting the camera");

    SensorTopicsRes responded_topics = smi_.startSensor("camera", requested_topics);
    std::string camera_topic = responded_topics.getOutputTopic("camera_data");

    TEMOTO_INFO_STREAM("Got camera on topic '" << camera_topic << "'");

    // Pass the camera topic to the output
    what_0_word_out = what_0_word_in;
    what_0_data_0_out = camera_topic;
    // -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
  }

  /*
   * Interface 2 body
   */
  void startInterface_2()
  {
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string what_0_word_out;
    std::string what_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // -------------------------------< USER CODE >-------------------------------

    // Initialize the context manager interface
    cmi_.initialize(this);

    TEMOTO_INFO(" Starting the hand tracker");

    // Build a gesture specifier
    // TODO: This shoud be done via gesture specifier helper class
    std::vector<temoto_2::GestureSpecifier> gesture_specifiers;
    temoto_2::GestureSpecifier gesture_specifier;
    gesture_specifier.dev = "device";
    gesture_specifier.type = "hand";
    gesture_specifiers.push_back(gesture_specifier);

    // Advertise the marker topic
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("/temoto_task_markers", 1);

    // Register the gesture request and bind a callback
    cmi_.getGesture(gesture_specifiers, &TaskStart::gestureCallback, this);

    // Pass the camera topic to the output
    what_0_word_out = what_0_word_in;
    what_0_data_0_out = "/temoto_task_markers";
    // -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
  }

  // Callback for processing gestures
  void gestureCallback(leap_motion_controller::Set gesture)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/wrist_3_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "hand_indicator";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and
    // CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in
    // the header
    marker.pose = gesture.left_hand.palm_pose.pose;
    //    double z = marker.pose.position.z;
    //    marker.pose.position.z = -0.5*marker.pose.position.y;
    //    marker.pose.position.y = -5*z;
    //    marker.pose.position.x *= -1.5;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.17;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    if (marker_pub_)
    {
      marker_pub_.publish(marker);
    }
    else
    {
      TASK_ERROR("Excepition ERROR kutsuge 112 cobra");
    }
  }

  ~TaskStart()
  {
    TEMOTO_INFO("TaskStart destructed");
  }

private:
  // Nodehandle
  ros::NodeHandle n_;

  // Create sensor manager interface object for accessing sensor manager
  sensor_manager::SensorManagerInterface<TaskStart> smi_;

  // Create context manager interface object for context manager manager
  context_manager::ContextManagerInterface<TaskStart> cmi_;

  // Marker publisher
  ros::Publisher marker_pub_;

  std::string class_name_ = "TaskStart";

  std::string task_alias;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskStart, TTP::BaseTask);
