#ifndef TEMOTO_SENSOR_SNOOPER_H
#define TEMOTO_SENSOR_SNOOPER_H

#include "common/base_subsystem.h"
#include "sensor_manager/sensor_info_database.h"
#include "rmp/config_synchronizer.h"
#include "temoto_2/ConfigSync.h"
#include "TTP/task_manager.h"

namespace sensor_manager
{

class SensorSnooper : public BaseSubsystem
{

  typedef std_msgs::String PayloadType;

public:

  SensorSnooper( BaseSubsystem* b
               , SensorInfoDatabase* sid);

  void advertiseSensor(SensorInfoPtr sensor_ptr) const;

  void advertiseLocalSensors() const;

  std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config);

private:

  void syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload);

  void startSnooping();


  rmp::ConfigSynchronizer<SensorSnooper, PayloadType> config_syncer_;

  SensorInfoDatabase* sid_;

  TTP::TaskManager action_engine_;

};

} // sensor_manager namespace

#endif
