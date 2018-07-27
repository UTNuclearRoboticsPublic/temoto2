#include "common/base_subsystem.h"
#include "sensor_manager/sensor_manager_services.h"
#include "sensor_manager/sensor_manager_servers.h"
#include "sensor_manager/sensor_info_database.h"
#include "sensor_manager/sensor_snooper.h"

using namespace sensor_manager;

class SensorManager : public BaseSubsystem
{
public:

  SensorManager()
  : BaseSubsystem("sensor_manager", error::Subsystem::SENSOR_MANAGER, __func__)
  , ss_(this, &sid_)
  , sms_(this, &sid_)
  {
    TEMOTO_INFO("Sensor Manager is good to go.");
  }

private:

  SensorInfoDatabase sid_;
  SensorSnooper ss_;
  SensorManagerServers sms_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_manager");

  // Create a SensorManager object
  SensorManager sm;

  //use single threaded spinner for global callback queue
  ros::spin();

//  ros::AsyncSpinner spinner(4); // Use 4 threads
//  spinner.start();
//  ros::waitForShutdown();

  return 0;
}
