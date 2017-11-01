#include "sensor_manager/sensor_manager.h"

using namespace sensor_manager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_manager");

  // Create a SensorManager object
  SensorManager sm;

  // Add a dummy sensor entry (For testing)
  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("temoto_tests", "hand"));
  sm.pkg_infos_.back()->addLaunchable({ "hand_tracker_0.launch", "/leap_motion_output" });

  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("leap_motion_controller", "hand"));
  sm.pkg_infos_.back()->addRunnable({ "leap_motion", "/leap_motion_output" });

  //sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("temoto_2", "hand"));
  //sm.pkg_infos_.back()->addRunnable({ "dummy_sensor", "/dummy_sensor_data" });


  // sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("usb_cam", "camera"));
  // sm.pkg_infos_back().addLaunchable({"usb_cam-test.launch", "/usb_cam/image_raw"});

  // camera on /dev/video1
  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_show", "camera"));
  sm.pkg_infos_.back()->addLaunchable({ "camera1.launch", "/usb_cam/image_raw" });

  // camera on /dev/video1
  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_show", "camera"));
  sm.pkg_infos_.back()->addLaunchable({ "camera0.launch", "/usb_cam/image_raw" });

//  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("pocket_sphinx", "speech"));
 // sm.pkg_infos_.back()->addLaunchable({ "camera0.launch", "/usb_cam/image_raw" });
 
  
  // set up terminal as backup input if the above should fail
  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("temoto_2", "speech"));
  sm.pkg_infos_.back()->addLaunchable({ "test_2.launch", "/terminal_text" });

  // Google's speech to text sensor
  sm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("google_speech_to_text", "speech"));
  sm.pkg_infos_.back()->addLaunchable({ "en-US.launch", "/stt/spoken_text" });

  //use single threaded spinner for global callback queue
   ros::spin();

//  ros::AsyncSpinner spinner(4); // Use 4 threads
//  spinner.start();
//  ros::waitForShutdown();

  return 0;
}
