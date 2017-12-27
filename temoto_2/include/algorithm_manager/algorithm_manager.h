#ifndef ALGORITHM_MANAGER_H
#define ALGORITHM_MANAGER_H

#include "algorithm_manager/algorithm_info.h"
#include "algorithm_manager/algorithm_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "common/base_subsystem.h"
#include "rmp/resource_manager.h"
#include "rmp/config_synchronizer.h"

#include "std_msgs/String.h"

namespace algorithm_manager
{

typedef std_msgs::String PayloadType;

class AlgorithmManager : public BaseSubsystem
{
public:

  AlgorithmManager();

  /**
   * @brief getName
   * @return
   */
  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  rmp::ResourceManager<AlgorithmManager> resource_manager_;

  rmp::ConfigSynchronizer<AlgorithmManager, PayloadType> config_syncer_;

  /**
   * @brief List of all locally defined algorithms.
   */
  std::vector<AlgorithmInfoPtr> local_algorithms_;

  /**
   * @brief List of all algorithms in remote managers.
   */
  std::vector<AlgorithmInfoPtr> remote_algorithms_;

  /**
   * @brief List of allocated algorithms
   */
  std::map<temoto_id::ID, AlgorithmInfoPtr> allocated_algorithms_;

  /**
   * @brief loadAlgorithmCb
   * @param req
   * @param res
   */
  void loadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req, temoto_2::LoadAlgorithm::Response& res);

  /**
   * @brief Called when algorithm is unloaded. Nothing to do here.
   * @return
   */
  void unloadAlgorithmCb(temoto_2::LoadAlgorithm::Request& req, temoto_2::LoadAlgorithm::Response& res);

  /**
   * @brief Called when algorithm is unloaded. Nothing to do here.
   * @return
   */
  void statusCb(temoto_2::ResourceStatus& srv);

  /**
   * @brief advertiseLocalAlgorithms
   */
  void advertiseAlgorithm(AlgorithmInfoPtr);

  /**
   * @brief advertiseLocalAlgorithms
   */
  void advertiseLocalAlgorithms();

  /**
   * @brief syncCb
   * @param msg
   * @param payload
   */
  void syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload);

  /**
   * @brief parseAlgorithms
   * @param config
   * @return
   */
  AlgorithmInfoPtrs parseAlgorithms(const YAML::Node& config);

  /**
   * @brief findAlgorithm
   * @param type
   * @param name
   * @param executable
   * @param algorithms
   * @return
   */
  AlgorithmInfoPtr findAlgorithm(temoto_2::LoadAlgorithm::Request req
                               , const std::vector<AlgorithmInfoPtr>& algorithms);

};

} // context_manager namespace

#endif
