#ifndef CLIENT_QUERY_H
#define CLIENT_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <utility>
#include "common/temoto_id.h"

namespace rmp
{
// class for storing resource requests and hold their bingdings to external servers
template <class ServiceMsgType, class Owner>
class ClientQuery : public BaseSubsystem
{
public:
  // special constructor for resource client
  ClientQuery(const ServiceMsgType& msg, Owner* owner)
    : BaseSubsystem(*owner, __func__), msg_(msg), owner_(owner), failed_(false)
  {
    this->log_group_ = "rmp." + this->log_group_;
  }

  void addInternalResource(temoto_id::ID resource_id, std::string& server_name)
  {
    // try to insert to map
    TEMOTO_DEBUG("Adding id:%d, server_name:'%s'", resource_id, server_name.c_str());
    auto ret = internal_resources_.emplace(resource_id, server_name);

    if (!ret.second)
    {
      throw CREATE_ERROR(error::Code::RMP_FATAL, "Not allowed to add internal resources with "
                                                 "identical ids.");
    }
  }

  // remove the internal resource from this query and return how many are still connected
  size_t removeInternalResource(temoto_id::ID resource_id)
  {
    auto it = internal_resources_.find(resource_id);
    if (it == internal_resources_.end())
    {
      throw CREATE_ERROR(error::Code::RMP_FATAL,
                         "Unable to remove the resource. Resource_id %s not found.", resource_id);
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
    TEMOTO_DEBUG_STREAM("    Query req:" << std::endl << msg_.request);
    TEMOTO_DEBUG_STREAM("    Query res:" << std::endl << msg_.response);
    TEMOTO_DEBUG_STREAM("    Internal resources:");
    for (auto& r : internal_resources_)
    {
      TEMOTO_DEBUG("     id:%d server_name:'%s'", r.first, r.second.c_str());
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

  Owner* owner_;

  ServiceMsgType msg_;  /// Store request and response, note that RMP specific fields (resource_id,
                        /// topic, ...) are related to first query and are not intended to be used
                        /// herein.
};
}

#endif
