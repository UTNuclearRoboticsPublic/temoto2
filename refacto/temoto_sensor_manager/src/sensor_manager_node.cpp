#include "temoto_core/common/base_subsystem.h"
#include "temoto_sensor_manager/sensor_manager_services.h"
#include "temoto_sensor_manager/sensor_manager_servers.h"
#include "temoto_sensor_manager/sensor_info_registry.h"
#include "temoto_sensor_manager/sensor_snooper.h"

#include <signal.h>

using namespace temoto_sensor_manager;

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
  , ss_(this, &sir_)
  , sms_(this, &sir_)
  {
    ss_.startSnooping();
    TEMOTO_INFO("Sensor Manager is good to go.");
  }

  ~SensorManager()
  {
    // "cout" instead of "TEMOTO_INFO" because otherwise it will print nothing
    std::cout << "Shutting down the Sensor Manager ..." << std::endl;
  }

private:

  /// Sensor Info Database
  SensorInfoRegistry sir_;

  /// Sensror Manager Servers
  SensorManagerServers sms_;

  /// Sensor Snooper
  SensorSnooper ss_;
};

/*
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_manager");

  // Create a SensorManager object
  SensorManager sm;

  //use single threaded spinner for global callback queue
  ros::spin();

  return 0;
}
