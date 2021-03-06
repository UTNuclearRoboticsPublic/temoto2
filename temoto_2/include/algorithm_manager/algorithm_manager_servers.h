#ifndef ALGORITHM_MANAGER_SERVERS_H
#define ALGORITHM_MANAGER_SERVERS_H

#include "common/base_subsystem.h"
#include "algorithm_manager/algorithm_info_registry.h"
#include "algorithm_manager/algorithm_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"

#include "std_msgs/String.h"
#include "common/temoto_id.h"

namespace algorithm_manager
{

typedef std_msgs::String PayloadType;

class AlgorithmManagerServers : public BaseSubsystem
{
public:
  /**
   * @brief AlgorithmManagerServers
   */
  AlgorithmManagerServers( BaseSubsystem* b, AlgorithmInfoRegistry* air );

  /**
   * @brief ~AlgorithmManagerServers
   */
  ~AlgorithmManagerServers();

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  /**
   * @brief Start node service
   * @param req
   * @param res
   * @return
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
   * @brief processTopics
   * @param req
   * @param res
   * @param load_process_msg
   * @param algorithm_ptr
   */
  void processTopics( std::vector<diagnostic_msgs::KeyValue>& req_topics
                    , std::vector<diagnostic_msgs::KeyValue>& res_topics
                    , temoto_2::LoadProcess& load_process_msg
                    , AlgorithmInfo& algorithm_info
                    , bool inputTopics);

  /// Algorithm Info Registry
  AlgorithmInfoRegistry* air_;

  ///  ros::ServiceServer list_devices_server_;
  rmp::ResourceManager<AlgorithmManagerServers> resource_manager_;

  /// List of allocated algorithms
  std::map<temoto_id::ID, AlgorithmInfo> allocated_algorithms_;

}; // AlgorithmManagerServers

} // algorithm_manager namespace

#endif
