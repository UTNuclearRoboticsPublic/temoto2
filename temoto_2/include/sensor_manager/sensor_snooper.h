#ifndef TEMOTO_SENSOR_SNOOPER_H
#define TEMOTO_SENSOR_SNOOPER_H

#include "common/base_subsystem.h"
#include "sensor_manager/sensor_info_database.h"
#include "rmp/config_synchronizer.h"
#include "temoto_2/ConfigSync.h"
#include "TTP/task_manager.h"

#include "ros/ros.h"

namespace sensor_manager
{

class SensorSnooper : public BaseSubsystem
{
  typedef std_msgs::String PayloadType;

public:

  SensorSnooper( BaseSubsystem* b, SensorInfoDatabase* sid);

  void advertiseSensor(SensorInfo& si) const;

  void advertiseLocalSensors() const;

  void startSnooping();

  std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config);

  ~SensorSnooper();

private:

  void syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload);

  void updateMonitoringTimerCb(const ros::TimerEvent &e);


  ros::NodeHandle nh_;

  rmp::ConfigSynchronizer<SensorSnooper, PayloadType> config_syncer_;

  SensorInfoDatabase* sid_;

  TTP::TaskManager action_engine_;

  ros::Timer update_monitoring_timer_;

};

} // sensor_manager namespace

#endif
