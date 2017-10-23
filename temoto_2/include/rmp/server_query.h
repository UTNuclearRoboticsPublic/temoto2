#ifndef RESOURCE_QUERY_H
#define RESOURCE_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>
#include "common/temoto_id.h"

namespace rmp
{
// class for storing resource requests and hold their bingdings to clients
template <class ServiceMsgType, class Owner>
class ServerQuery
{
public:
  ServerQuery()
  {
  } 
  // special constructor for resource server
  ServerQuery(const typename ServiceMsgType::Request& req, Owner* owner) : owner_(owner)
  {
    msg_.request = req;  // response part is set after executing owners callback
    rmp_log_prefix_ = owner->getName() + "::RMP::ServerQuery";
  }

  void addExternalResource(temoto_id::ID external_resource_id, const std::string& status_topic)
  {
    external_resources_.emplace(external_resource_id, status_topic);
  }

  // remove the external client from this query and return how many are still connected
  size_t removeExternalResource(temoto_id::ID resource_id)
  {
    /// Try to erase resource_id from external client map.
    external_resources_.erase(resource_id);
    return external_resources_.size();
  }

  // Check if external client with given resource_id is attached to this query.
  bool externalResourceExists(temoto_id::ID resource_id) const
  {
    return external_resources_.find(resource_id) != external_resources_.end();
  }

  // Check if external client with given resource_id is attached to this query.
  bool internalResourceExists(temoto_id::ID resource_id) const
  {
    auto found_r = find_if(internal_resources_.begin(), internal_resources_.end(),
                           [&](const std::pair<std::string, std::set<temoto_id::ID>>& p) -> bool {
                             return p.second.find(resource_id) != p.second.end();
                           });
    return found_r != internal_resources_.end();
  }

  void addInternalResource(std::string client_name, temoto_id::ID resource_id)
  {
    std::string prefix = rmp_log_prefix_ + __func__;
    // try to insert to map
    std::set<temoto_id::ID> s;
    s.insert(resource_id);
    auto ret = internal_resources_.emplace(client_name, s);

    if (ret.second == false)
    {
      // client already exists, add current id to its set
      auto ins_ret = ret.first->second.insert(resource_id);

      if (ins_ret.second == false)
      {
        ROS_ERROR_NAMED(log_name_, "An extreme badness has happened. Somebody tried to bind same "
                                   "resource twice to a resource_server.");
      }
    }
  }

  const ServiceMsgType& getMsg() const
  {
    return msg_;
  }

  const std::map<std::string, std::set<temoto_id::ID>>& getInternalResources() const
  {
    return internal_resources_;
  }

  const std::map<temoto_id::ID, std::string>& getExternalResources() const
  {
    return external_resources_;
  }

  void setMsgResponse(const typename ServiceMsgType::Response& res)
  {
    msg_.response = res;
  }

private:
  std::string rmp_log_prefix_;
  std::string log_name_ = "rmp";

  Owner* owner_;

  // unique client name is mapped to a set of resource ids
  std::map<std::string, std::set<temoto_id::ID>> internal_resources_;

  // represent external clients by external_resource_id and status_topic
  std::map<temoto_id::ID, std::string> external_resources_;

  ServiceMsgType msg_;  /// Store request and response, note that RMP specific fields (resource_id,
                        /// topic, ...) are related to first query and are not intended to be used
                        /// herein.
};
}

#endif
