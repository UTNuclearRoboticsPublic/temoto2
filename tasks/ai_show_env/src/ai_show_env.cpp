// Things that have to be included
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"
#include "output_manager/output_manager_interface.h"

// First implementaton
class ShowEnv : public TTP::BaseTask
{
public:
  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented /START
   * * * * * * * * * * * * * * * * * * * * * * * * */

  ShowEnv()
  {
    // Do something here if needed
    TEMOTO_INFO("ShowEnv constructed");
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

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // --------------------------------< USER CODE >-------------------------------

    //Initialize the interfaces
    omi_.initialize(this);
    smi_.initialize(this);

    // Assign a custom status update callback
    smi_.registerUpdateCallback(&ShowEnv::statusUpdateCb);

    // Request a depthcamera
    SensorTopicsReq requested_topics;
    requested_topics.addOutputTopicType("camera_data");
    requested_topics.addOutputTopicType("camera_info");

    TEMOTO_INFO("Starting the depth camera");

    SensorTopicsRes responded_topics = smi_.startSensor("depth camera", requested_topics);
    depth_camera_topic = responded_topics.getOutputTopic("camera_data");

    // Show the camera image in rviz
    TEMOTO_INFO("Showing depth camera feed in RViz @ '%s' topic", depth_camera_topic.c_str());

    // Show the image in rviz
    omi_.showInRviz("depth image", depth_camera_topic);

    // Load a dummy plugin
    omi_.showInRviz("marker");

    // --------------------------------</ USER CODE >-------------------------------

  }

  /*
   * A custom implementation for a resource status callback
   */
  void statusUpdateCb( bool status)
  {
    // Do nothing if the status message is not about a sensor failure
    if (status)
    {
      return;
    }

    TEMOTO_WARN_STREAM("Received an update message: " << status);

    switch(active_device)
    {
      /*
       * Load a depth camera
       */
      case ActiveDevice::CAMERA:
      {
        active_device = ActiveDevice::DEPTHCAMERA;

        // Request a depthcamera
        SensorTopicsReq requested_topics;
        requested_topics.addOutputTopicType("camera_data");
        requested_topics.addOutputTopicType("camera_info");

        TEMOTO_INFO("Starting the depth camera");

        SensorTopicsRes responded_topics = smi_.startSensor("depth camera", requested_topics);
        depth_camera_topic = responded_topics.getOutputTopic("camera_data");

        // Show the camera image in rviz
        TEMOTO_INFO("Showing depth camera feed in RViz @ '%s' topic", depth_camera_topic.c_str());

        // Show the image in rviz
        omi_.hideInRviz("image", camera_topic);
        omi_.showInRviz("depth image", depth_camera_topic);

        break;
      }

      /*
       * Load a LIDAR
       */
      case ActiveDevice::DEPTHCAMERA:
      {
        active_device = ActiveDevice::LIDAR;

        // Request a depthcamera
        SensorTopicsReq requested_topics;
        requested_topics.addOutputTopicType("lidar_data");

        TEMOTO_INFO("Starting the lidar");

        SensorTopicsRes responded_topics = smi_.startSensor("lidar", requested_topics);
        lidar_topic = responded_topics.getOutputTopic("lidar_data");

        // Show the camera image in rviz
        TEMOTO_INFO("Showing lidar feed in RViz @ '%s' topic", lidar_topic.c_str());

        // Show the image in rviz
        omi_.hideInRviz("depth image", depth_camera_topic);
        omi_.showInRviz("laser scan", lidar_topic);

        break;
      }

      /*
       * Load a camera
       */
      case ActiveDevice::LIDAR:
      {
        active_device = ActiveDevice::CAMERA;

        // Request a camera
        SensorTopicsReq requested_topics;
        requested_topics.addOutputTopicType("camera_data");
        requested_topics.addOutputTopicType("camera_info");

        TEMOTO_INFO("Starting the camera");

        SensorTopicsRes responded_topics = smi_.startSensor("camera", requested_topics);
        camera_topic = responded_topics.getOutputTopic("camera_data");

        // Show the camera image in rviz
        TEMOTO_INFO("Showing camera feed in RViz @ '%s' topic", camera_topic.c_str());

        // Show the image in rviz
        omi_.hideInRviz("laser scan", lidar_topic);
        omi_.showInRviz("image", camera_topic);

        break;
      }
    }
  }

  ~ShowEnv()
  {
    TEMOTO_INFO("ShowEnv destructed");
  }

private:
  // Nodehandle
  ros::NodeHandle n_;

  // Create sensor manager interface object for accessing sensor manager
  sensor_manager::SensorManagerInterface<ShowEnv> smi_;

  // Create context manager interface object for context manager manager
  context_manager::ContextManagerInterface<ShowEnv> cmi_;

  // Create sensor manager interface object for accessing sensor manager
  output_manager::OutputManagerInterface<ShowEnv> omi_;

  // Create an enum for storing the knowledge about the active device
  enum class ActiveDevice {DEPTHCAMERA, CAMERA, LIDAR};

  ActiveDevice active_device = ActiveDevice::DEPTHCAMERA;

  std::string camera_topic, depth_camera_topic, lidar_topic;

  std::string task_alias;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(ShowEnv, TTP::BaseTask);
