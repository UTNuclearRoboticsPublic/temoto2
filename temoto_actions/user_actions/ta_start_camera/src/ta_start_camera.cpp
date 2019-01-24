/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "temoto_nlp/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"
#include <visualization_msgs/Marker.h>

// First implementaton
class TaStartCamera : public temoto_nlp::BaseTask
{
public:
  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented /START
   * * * * * * * * * * * * * * * * * * * * * * * * */

  TaStartCamera()
  {
    // Do something here if needed
    TEMOTO_INFO("TaStartCamera constructed");
  }

  // startTask with arguments
  void startTask(temoto_nlp::TaskInterface task_interface)
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
    }
    // </ AUTO-GENERATED, DO NOT MODIFY >
  }

  std::vector<temoto_nlp::Subject> getSolution()
  {
    return output_subjects;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented / END
   * * * * * * * * * * * * * * * * * * * * * * * * */

  /*
   * Interface 1 body
   */
  void startInterface_0()
  {
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string what_0_word_out;
    std::string what_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // -------------------------------< USER CODE >-------------------------------

    // Initialize the sensor manager interface
    smi_.initialize(this);

    SensorTopicsReq requested_topics;
    requested_topics.addOutputTopic("camera_data", "/cam_data_topic");
    requested_topics.addOutputTopic("camera_info", "/cam_info_topic");

    TEMOTO_INFO(" Starting the camera");

    SensorTopicsRes responded_topics = smi_.startSensor("camera", requested_topics);
    std::string camera_topic = responded_topics.getOutputTopic("camera_data");

    TEMOTO_INFO_STREAM("Got camera on topic '" << camera_topic << "'");

    // Pass the camera topic to the output
    what_0_word_out = what_0_word_in;
    what_0_data_0_out = camera_topic;
    // -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    temoto_nlp::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
  }

  ~TaStartCamera()
  {
    TEMOTO_INFO("TaStartCamera destructed");
  }

private:
  // Nodehandle
  ros::NodeHandle n_;

  // Create sensor manager interface object for accessing sensor manager
  sensor_manager::SensorManagerInterface<TaStartCamera> smi_;

  // Create context manager interface object for context manager manager
  context_manager::ContextManagerInterface<TaStartCamera> cmi_;

  // Marker publisher
  ros::Publisher marker_pub_;

  std::string class_name_ = "TaStartCamera";

  std::string task_alias;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaStartCamera, temoto_nlp::BaseTask);
