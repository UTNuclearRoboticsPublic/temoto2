#include "common/base_subsystem.h"
#include "algorithm_manager/algorithm_manager_services.h"
#include "algorithm_manager/algorithm_manager_servers.h"
#include "algorithm_manager/algorithm_info_registry.h"
#include "algorithm_manager/algorithm_snooper.h"

#include <signal.h>

using namespace algorithm_manager;

/**
 * @brief The Algorithm Manager maintains 3 components of this subsystem
 */
class AlgorithmManager : public BaseSubsystem
{
public:

  /**
   * @brief Constructor
   */
  AlgorithmManager()
  : BaseSubsystem("algorithm_manager", error::Subsystem::SENSOR_MANAGER, __func__)
  , as_(this, &air_)
  , ams_(this, &air_)
  {
    as_.startSnooping();
    TEMOTO_INFO("Algorithm Manager is good to go.");
  }

  ~AlgorithmManager()
  {
    // "cout" instead of "TEMOTO_INFO" because otherwise it will print nothing
    std::cout << "Shutting down the Algorithm Manager ..." << std::endl;
  }

private:

  /// Algorithm Info Database
  AlgorithmInfoRegistry air_;

  /// Sensror Manager Servers
  AlgorithmManagerServers ams_;

  /// Algorithm Snooper
  AlgorithmSnooper as_;
};

/*
 * Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "algorithm_manager");

  // Create a AlgorithmManager object
  AlgorithmManager sm;

  //use single threaded spinner for global callback queue
  ros::spin();

  return 0;
}
