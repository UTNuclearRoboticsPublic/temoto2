#ifndef ALGORITHM_MANAGER_H
#define ALGORITHM_MANAGER_H

#include "algorithm_manager/algorithm_info.h"
#include "algorithm_manager/algorithm_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/rmp/config_synchronizer.h"

#include "std_msgs/String.h"

namespace algorithm_manager
{

typedef std_msgs::String PayloadType;

class AlgorithmManager : public temoto_core::BaseSubsystem
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

  temoto_core::rmp::ResourceManager<AlgorithmManager> resource_manager_;

  temoto_core::rmp::ConfigSynchronizer<AlgorithmManager, PayloadType> config_syncer_;

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
  std::map<temoto_core::temoto_id::ID, AlgorithmInfoPtr> allocated_algorithms_;

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
  void statusCb(temoto_core::ResourceStatus& srv);

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
  void syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload);

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

  /**
   * @brief remapArguments
   * @param req
   * @param res
   * @param load_process_msg
   * @param algorithm_ptr
   */
  void remapArguments(std::vector<diagnostic_msgs::KeyValue>& req_topics,
                      std::vector<diagnostic_msgs::KeyValue>& res_topics,
                      temoto_2::LoadProcess& load_process_msg,
                      AlgorithmInfoPtr algorithm_ptr,
                      bool inputTopics);
};

} // temoto_context_manager namespace

#endif
