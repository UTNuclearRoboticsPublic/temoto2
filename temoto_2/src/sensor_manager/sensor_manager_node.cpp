#include "common/base_subsystem.h"
#include "sensor_manager/sensor_manager_services.h"
#include "sensor_manager/sensor_manager_servers.h"
#include "sensor_manager/sensor_info_database.h"
#include "sensor_manager/sensor_snooper.h"

#include <signal.h>

using namespace sensor_manager;

/**
 * @brief The Sensor Manager maintains 3 components of this subsystem
 */
class SensorManager : public BaseSubsystem
{
public:

  /**
   * @brief Constructor
   */
  SensorManager()
  : BaseSubsystem("sensor_manager", error::Subsystem::SENSOR_MANAGER, __func__)
  , ss_(this, &sid_)
  , sms_(this, &sid_)
  {
    ss_.startSnooping();
    TEMOTO_INFO("Sensor Manager is good to go.");
  }

  ~SensorManager()
  {
    std::cout << "Shutting down the Sensor Manager ..." << std::endl;
  }

private:

  /// Sensor Info Database
  SensorInfoDatabase sid_;

  /// Sensror Manager Servers
  SensorManagerServers sms_;

  /// Sensor Snooper
  SensorSnooper ss_;
};

// Create a pointer to sensor manager. This is used for custom SIGINT handling
SensorManager* smp;

//void sigintHandler (int signum)
//{
//  ros::shutdown();

//  // Stop the sensor snooper
//  delete smp;
//}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_manager");

  // Create a SensorManager object
  //smp = new SensorManager;

  SensorManager sm;

  //signal(SIGINT, sigintHandler);

  //use single threaded spinner for global callback queue
  ros::spin();

  return 0;
}
