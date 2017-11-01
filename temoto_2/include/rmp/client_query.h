#ifndef CLIENT_QUERY_H
#define CLIENT_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <utility>
#include "common/temoto_id.h"
#include "rmp/log_macros.h"

namespace rmp
{
// class for storing resource requests and hold their bingdings to external servers
template <class ServiceMsgType, class Owner>
class ClientQuery
{
public:
  ClientQuery() : owner_(NULL), failed_(false)
  {
  }
  // special constructor for resource client
  ClientQuery(const ServiceMsgType& msg, Owner* owner) : msg_(msg), owner_(owner), failed_(false)
  {
    log_class_ = "rmp/ClientQuery";
    log_subsys_ = owner_->getName();
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
  }

  void addInternalResource(temoto_id::ID resource_id, std::string& server_name)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
    // try to insert to map
    RMP_DEBUG("%s id:%d, server_name:'%s'", prefix.c_str(), resource_id,
                    server_name.c_str());
    auto ret = internal_resources_.emplace(resource_id, server_name);

    if (!ret.second)
    {
      RMP_ERROR("%s Not allowed to add internal resources with "
                "identical ids.", prefix.c_str());
    }
  }

  // remove the internal resource from this query and return how many are still connected
  size_t removeInternalResource(temoto_id::ID resource_id)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
    auto it = internal_resources_.find(resource_id);
    if (it == internal_resources_.end())
    {
      RMP_ERROR("%s Resource_id not found!", prefix.c_str());
    }
    std::string caller_name = it->second;

    /// Erase resource_id from internal resource map.
    internal_resources_.erase(it);

    /// Count how many internal connections we have left with the erased caller.
    size_t cnt = count_if(internal_resources_.begin(), internal_resources_.end(),
                          [&](const std::pair<temoto_id::ID, std::string>& r) -> bool {
                            return r.second == caller_name;
                          });

    return cnt;
  }

  // Check if given internal resource_id is attached to this query.
  bool internalResourceExists(temoto_id::ID resource_id) const
  {
    return internal_resources_.find(resource_id) != internal_resources_.end();
  }

  const ServiceMsgType& getMsg() const
  {
    return msg_;
  }

  const temoto_id::ID getExternalId() const
  {
    return msg_.response.rmp.resource_id;
  }

  void debug()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
    RMP_DEBUG_STREAM(prefix << "    Query req:" << std::endl << msg_.request);
    RMP_DEBUG_STREAM(prefix << "    Query res:" << std::endl << msg_.response);
    RMP_DEBUG_STREAM(prefix << "    Internal resources:");
    for (auto& r : internal_resources_)
    {
      RMP_DEBUG("%s      id:%d server_name:'%s'",prefix.c_str(), r.first, r.second.c_str());
    }
  }

  const std::map<temoto_id::ID, std::string> getInternalResources() const
  {
    return internal_resources_;
  }

  bool failed_;

private:
  // internal resource ids and their callers name
  std::map<temoto_id::ID, std::string> internal_resources_;

  std::string log_class_, log_subsys_;
  std::string rmp_log_prefix_;
  Owner* owner_;

  ServiceMsgType msg_;  /// Store request and response, note that RMP specific fields (resource_id,
                        /// topic, ...) are related to first query and are not intended to be used
                        /// herein.
};
}

#endif
