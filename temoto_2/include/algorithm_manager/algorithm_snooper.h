#ifndef TEMOTO_ALGORITHM_SNOOPER_H
#define TEMOTO_ALGORITHM_SNOOPER_H

#include "common/base_subsystem.h"
#include "algorithm_manager/algorithm_info_registry.h"
#include "rmp/config_synchronizer.h"
#include "temoto_2/ConfigSync.h"
#include "TTP/task_manager.h"

#include "ros/ros.h"

namespace algorithm_manager
{

class AlgorithmSnooper : public BaseSubsystem
{
  typedef std_msgs::String PayloadType;

public:

  AlgorithmSnooper( BaseSubsystem* b, AlgorithmInfoRegistry* sid);

  void advertiseAlgorithm(AlgorithmInfo& si) const;

  void advertiseLocalAlgorithms() const;

  void startSnooping();

  std::vector<AlgorithmInfoPtr> parseAlgorithms(const YAML::Node& config);

private:

  void syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload);

  void updateMonitoringTimerCb(const ros::TimerEvent &e);


  ros::NodeHandle nh_;

  rmp::ConfigSynchronizer<AlgorithmSnooper, PayloadType> config_syncer_;

  AlgorithmInfoRegistry* sid_;

  TTP::TaskManager action_engine_;

  ros::Timer update_monitoring_timer_;

};

} // algorithm_manager namespace

#endif
