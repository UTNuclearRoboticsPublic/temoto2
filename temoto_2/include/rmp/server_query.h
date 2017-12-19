#ifndef RESOURCE_QUERY_H
#define RESOURCE_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>
#include "common/temoto_id.h"
#include "rmp/log_macros.h"

namespace rmp
{
// class for storing resource requests and hold their bingdings to clients
template <class ServiceMsgType, class Owner>
class ServerQuery
{
public:
  ServerQuery() : failed_(false), owner_(NULL)
  {
  } 
  // special constructor for resource server
  ServerQuery(const typename ServiceMsgType::Request& req, Owner* owner) : owner_(owner), failed_(false)
  {
    log_class_ = "rmp/ServerQuery";
    log_subsys_ = owner_->getName();
    msg_.request = req;  // response part is set after executing owners callback
  }

  void addExternalResource(temoto_id::ID external_resource_id, const std::string& status_topic)
  {
    external_resources_.emplace(external_resource_id, status_topic);
  }

  // remove the external client from this query and return how many are still connected
  size_t removeExternalResource(temoto_id::ID external_resource_id)
  {
    /// Try to erase resource_id from external client map.
    external_resources_.erase(external_resource_id);
    return external_resources_.size();
  }

  // Check if external connection with given resource_id is attached to this query.
  bool externalResourceExists(temoto_id::ID external_resource_id) const
  {
    return external_resources_.find(external_resource_id) != external_resources_.end();
  }

  // Check if external client with given resource_id is attached to this query.
  bool internalResourceExists(temoto_id::ID internal_resource_id) const
  {
    return internal_resources_.find(internal_resource_id) != internal_resources_.end();
  }

  void addInternalResource(temoto_id::ID internal_resource_id)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");

    auto ret = internal_resources_.emplace(internal_resource_id);
    if (ret.second == false)
    {
      // resource already exists, something that should never happen...
      RMP_ERROR("%s An extreme badness has happened. Somebody tried to bind same "
                "resource twice to a resource_server.",
                prefix.c_str());
    }
  }

  const ServiceMsgType& getMsg() const
  {
    return msg_;
  }

  const std::set<temoto_id::ID>& getInternalResources() const
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


  //TODO: REMOVE ME
  bool failed_;

private:
  std::string log_class_, log_subsys_;

  Owner* owner_;

  temoto_id::ID internal_resource_id;

  // ID's of internally linked clients. Those are added automatically when call()
  // function is called from owner's load callback.
  std::set<temoto_id::ID> linked_resources_;

  // represent external clients by external_resource_id and status_topic
  std::map<temoto_id::ID, std::string> external_resources_;

  ServiceMsgType msg_;  /// Store request and response, note that RMP specific fields (resource_id,
                        /// topic, ...) are related to first query and are not intended to be used
                        /// herein.
};
}

#endif
