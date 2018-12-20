#ifndef TEMOTO_SENSOR__MANAGER_SENSOR_SNOOPER_H
#define TEMOTO_SENSOR__MANAGER_SENSOR_SNOOPER_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/rmp/config_synchronizer.h"
#include "temoto_sensor_manager/sensor_info_registry.h"
#include "TTP/task_manager.h"

#include "ros/ros.h"

namespace temoto_sensor_manager
{

/**
 * @brief Sensor Snooper is responsible of finding sensor devices and updating other sensor snoopers
 * about available sensors (in a system where multiple instances of temoto are running).
 * Sensor Snooper uses snooping agents (temoto actions) to discover sensor devices.
 */
class SensorSnooper : public temoto_core::BaseSubsystem
{
  /**
   * @brief Defines what kind of data type is used in sensor info synchronization messages.
   */
  typedef std_msgs::String PayloadType;

public:

  /**
   * @brief Constructor
   * @param b Pointer to the parent subsystem that embeds this object.
   * @param sir Pointer to the Sensor Info Registry.
   */
  SensorSnooper( temoto_core::BaseSubsystem* b, SensorInfoRegistry* sir);

  /**
   * @brief Advertises a sensor to other sensor snoopers.
   * @param si Sensor to advertise.
   */
  void advertiseSensor(SensorInfo& si) const;

  /**
   * @brief Advertises all sensors in the local system.
   */
  void advertiseLocalSensors() const;

  /**
   * @brief Executes the sensor snooping process. Utilizes Temoto actions as snooping agents.
   */
  void startSnooping();

  /**
   * @brief A helper function that is used for converting sensor yaml descriptions to sensor info
   * objects.
   * @param config Sensor Info in a yaml node format.
   * @return
   */
  std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config);

  /// Destructor
  ~SensorSnooper();

private:

  /**
   * @brief A callback function that is called when other instance of temoto has advertised
   * its sensors.
   * @param msg Incoming message
   * @param payload Data portion of the message
   */
  void syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload);

  /**
   * @brief A timer event callback function which checks if local sensor info entries have been
   * updated and if so, then advertises local sensors via #advertiseSensor.
   * @param e
   */
  void updateMonitoringTimerCb(const ros::TimerEvent &e);

  /// NodeHandle for the timer
  ros::NodeHandle nh_;

  /// Object that handles sensor info syncronization.
  temoto_core::rmp::ConfigSynchronizer<SensorSnooper, PayloadType> config_syncer_;

  /// Pointer to a central Sensor Info Registry object.
  SensorInfoRegistry* sir_;

  /// Used for managing snooper agents.
  TTP::TaskManager action_engine_;

  /**
   * @brief Timer for checking local sensor info updates (timer event will trigger the #updateMonitoringTimerCb).
   * The local sensor info objects are asynchronously updated/created by snooper agents and this timer
   * is responsible for checking if the updates are advertised to other instances of temoto.
   */
  ros::Timer update_monitoring_timer_;

};

} // sensor_manager namespace

#endif
