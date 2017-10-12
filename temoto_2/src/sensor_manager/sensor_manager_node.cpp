#include "sensor_manager/sensor_manager.h"

using namespace sensor_manager;

int main (int argc, char **argv)
{

    ros::init (argc, argv, "sensor_manager");

    // Create a SensorManager object
    SensorManager sensorManager;

    // Add a dummy sensro entry (For testing)
    sensorManager.pkgInfoList_.push_back (package_info("leap_motion_controller", "hand"));
    sensorManager.pkgInfoList_back().addRunnable({"leap_motion", "/leap_motion_output"});

    sensorManager.pkgInfoList_.push_back (package_info("temoto_2", "hand"));
    sensorManager.pkgInfoList_back().addRunnable({"dummy_sensor", "/dummy_sensor_data"});

    sensorManager.pkgInfoList_.push_back (package_info("temoto_2", "text"));
    sensorManager.pkgInfoList_.back().addLaunchable({"test_2.launch", "/human_chatter"});

   // sensorManager.pkgInfoList_.push_back (package_info("usb_cam", "camera"));
   // sensorManager.pkgInfoList_back().addLaunchable({"usb_cam-test.launch", "/usb_cam/image_raw"});

    sensorManager.pkgInfoList_.push_back (package_info("task_take_picture", "camera"));
    sensorManager.pkgInfoList_back().addLaunchable({"camera1.launch", "/usb_cam/image_raw"});

    sensorManager.pkgInfoList_.push_back (package_info("task_take_picture", "camera"));
    sensorManager.pkgInfoList_back().addLaunchable({"camera0.launch", "/usb_cam/image_raw"});

    ros::spin();

    return 0;
}
