#ifndef RESOURCE_SERVER_H
#define RESOURCE_SERVER_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "common/tools.h"
#include "rmp/base_resource_server.h"
#include "rmp/server_query.h"
#include "ros/callback_queue.h"
#include <mutex>

namespace rmp
{
template <class ServiceType, class Owner>
class ResourceServer : public BaseResourceServer<Owner>
{
public:
  typedef void (Owner::*LoadCbFuncType)(typename ServiceType::Request&,
                                        typename ServiceType::Response&);
  typedef void (Owner::*UnloadCbFuncType)(typename ServiceType::Request&,
                                          typename ServiceType::Response&);

  ResourceServer(std::string name, LoadCbFuncType load_cb, UnloadCbFuncType unload_cb, Owner* owner,
                 ResourceManager<Owner>& resource_manager)
    : BaseResourceServer<Owner>(name, resource_manager)
    , load_callback_(load_cb)
    , unload_callback_(unload_cb)
    , owner_(owner)
    , load_spinner_(2, &load_cb_queue_)

  {
    std::string prefix = common::generateLogPrefix(owner->getName(), log_name_, "");
    std::string rm_name = this->resource_manager_.getName();
    ros::AdvertiseServiceOptions load_service_opts =
        ros::AdvertiseServiceOptions::create<ServiceType>(
            rm_name + "/" + this->name_,
            boost::bind(&ResourceServer<ServiceType, Owner>::wrappedLoadCallback, this, _1, _2),
            ros::VoidPtr(), &this->load_cb_queue_);
    load_server_ = nh_.advertiseService(load_service_opts);
    load_spinner_.start();
    ROS_DEBUG_NAMED(log_name_, "%s ResourceServer constructed, listening on '%s'.", prefix.c_str(),
                    this->load_server_.getService().c_str());
  }

  ~ResourceServer()
  {
    load_spinner_.stop();
    std::string prefix = common::generateLogPrefix(owner_->getName(), log_name_, "");
    ROS_DEBUG_NAMED(log_name_, "%s ResourceServer destroyed.", prefix.c_str());
  }

  void registerInternalResource(std::string client_name, temoto_id::ID resource_id)
  {
    std::string prefix = common::generateLogPrefix(owner_->getName(), log_name_, "");
    ROS_DEBUG_NAMED(log_name_, "%s Trying to register id %d.", prefix.c_str(), resource_id);
    if (!queries_.size())
    {
      ROS_ERROR_NAMED(log_name_, "%s Failed because queries_ is empty.", prefix.c_str());
      return;
    }
    queries_.back().addInternalResource(client_name, resource_id);
  }

  bool wrappedLoadCallback(typename ServiceType::Request& req, typename ServiceType::Response& res)
  {
    std::string prefix = common::generateLogPrefix(owner_->getName(), log_name_, "");
    ROS_DEBUG_NAMED(log_name_, "%s Got query with status_topic: '%s'.", prefix.c_str(),
                    req.rmp.status_topic.c_str());

    if (!owner_)
    {
      ROS_ERROR_NAMED(log_name_, "%s ResourceServer Owner is NULL. Query aborted.", prefix.c_str());
      return true;
    }

    // generate new external id for the resource
    temoto_id::ID ext_resource_id = res_id_manager_.generateID();
    ROS_DEBUG_NAMED(log_name_, "%s Generated external id: '%d'.", prefix.c_str(), ext_resource_id);

    // lock the queries
    waitForLock(queries_mutex_);

    // New or existing query? Check it out with this hi-tec lambda function :)
    auto found_query = std::find_if(queries_.begin(), queries_.end(),
                                    [&req](const ServerQuery<ServiceType, Owner>& query) -> bool {
                                      return query.getMsg().request == req;
                                    });

    if (found_query == queries_.end())
    {
      // generate new internal id, and give it to owners callback.
      // with this owner can send status messages later when necessary
      temoto_id::ID int_resource_id = this->resource_manager_.generateInternalID();
      res.rmp.resource_id = int_resource_id;
      ROS_DEBUG_NAMED(log_name_, "%s New query, internal id: '%d'.", prefix.c_str(),
                      int_resource_id);

      // equal message not found from queries_, add new query
      queries_.emplace_back(req, owner_);
      queries_.back().addExternalResource(ext_resource_id, req.rmp.status_topic);

      // register owner as a nonamed client
      std::string client_name = "";
      queries_.back().addInternalResource(client_name, int_resource_id);

      // set this server active in resource manager
      // when a client call is made from callback, the binding between active server
      // and the new loaded resources can be made automatically
      waitForLock(active_server_mutex_);
      this->activateServer();

      // call owner's registered callback and release the lock during the callback so that owner is
      // able to use rmp inside the callback
      queries_mutex_.unlock();
      (owner_->*load_callback_)(req, res);
      waitForLock(queries_mutex_);

      // restore active server to NULL in resource manager
      this->deactivateServer();
      active_server_mutex_.unlock();

      ROS_DEBUG_NAMED(log_name_, "%s Resumed from owners callback", prefix.c_str());
      // verify that our query is still on the list
      auto q_it = std::find_if(queries_.begin(), queries_.end(),
                               [&](const ServerQuery<ServiceType, Owner>& q) -> bool {
                                 return q.internalResourceExists(int_resource_id);
                               });
      if (q_it != queries_.end())
      {
        // update the query with the response message filled in the callback
        q_it->setMsgResponse(res);
      }
      else
      {
        res.rmp.code = status_codes::FAILED;
        res.rmp.message += " Could not create resource.";
        ROS_ERROR_NAMED(log_name_, "%s Query got missing during owners callback, oh well...",
                        prefix.c_str());
      }
    }
    else
    {
      // found equal request, simply reqister this in the query
      // and respond with unique resoure_id.
      ROS_DEBUG_NAMED(log_name_, "%s Existing query, linking to the found query.", prefix.c_str());
      queries_.back().addExternalResource(ext_resource_id, req.rmp.status_topic);
    }

    // release the queries lock
    queries_mutex_.unlock();

    return true;
  }

  // This function is called from resource manager when /unload request arrives
  // (e.g. when some external client is being destroyed)
  // We look up the query that contains given external resource id and send unload to all internal
  // clients in the same query

  void unloadResource(temoto_2::UnloadResource::Request& req,
                      temoto_2::UnloadResource::Response& res)
  {
    std::string prefix = common::generateLogPrefix(owner_->getName(), log_name_, "");
    // find first query that contains resource that should be unloaded
    const temoto_id::ID resource_id = req.resource_id;
    waitForLock(queries_mutex_);
    const auto found_query_it =
        std::find_if(queries_.begin(), queries_.end(),
                     [resource_id](const ServerQuery<ServiceType, Owner>& query) -> bool {
                       return query.externalResourceExists(resource_id);
                     });
    if (found_query_it != queries_.end())
    {
      ROS_DEBUG_NAMED(log_name_, "%s Query with ext id %d was found", prefix.c_str(), resource_id);
      ROS_DEBUG_NAMED(log_name_, "%s internal resource count: %lu", prefix.c_str(),
                      found_query_it->getInternalResources().size());

      // Query found, try to remove client from it.
      size_t resources_left = found_query_it->removeExternalResource(req.resource_id);
      if (resources_left <= 0)
      {
        // last resource removed, execute owner's unload callback and remove the query from our list
        typename ServiceType::Request orig_req = found_query_it->getMsg().request;
        typename ServiceType::Response orig_res = found_query_it->getMsg().response;
        (owner_->*unload_callback_)(orig_req, orig_res);

        /// TODO: Do or do not something with the response part?

        // Send unload command to all internal clients...
        for (auto& map_el : found_query_it->getInternalResources())
        {
          //
          if (map_el.first == "")
          {
            // do not unload owner's resources automatically.
            // resources that were loaded outside of load callback are not RMP responsibility
            continue;
          }
          for (auto& set_el : map_el.second)
          {
            this->resource_manager_.unloadClientResource(set_el);
          }
        }

        // Finally, remove the found query.
        queries_.erase(found_query_it);
      }
    }
    queries_mutex_.unlock();
  }

  bool internalResourceExists(temoto_id::ID resource_id) const
  {
    auto found_q = find_if(queries_.begin(), queries_.end(),
                           [&](const ServerQuery<ServiceType, Owner>& q) -> bool {
                             return q.internalResourceExists(resource_id);
                           });
    return found_q != queries_.end();
  }

  std::vector<std::pair<temoto_id::ID, std::string>>
  getExternalResources(temoto_id::ID internal_resource_id)
  {
    waitForLock(queries_mutex_);

    std::vector<std::pair<temoto_id::ID, std::string>> ext_resources;
    for (const auto& q : queries_)
    {
      if (q.internalResourceExists(internal_resource_id))
      {
        for (auto resource : q.getExternalResources())
        {
          ext_resources.push_back(resource);
        }
      }
    }
    queries_mutex_.unlock();
    return ext_resources;
  }

  void waitForLock(std::mutex& m)
  {
    std::string prefix = common::generateLogPrefix(owner_->getName(), log_name_, "");
    while (!m.try_lock())
    {
      ROS_DEBUG_NAMED(log_name_, "%s Waiting for lock()", prefix.c_str());
      ros::Duration(0.01).sleep();  // sleep for few ms
    }
    // ROS_DEBUG_NAMED(log_name_, "%s Obtained lock()", prefix.c_str());
  }

private:
  const std::string log_name_ = "rmp";
  Owner* owner_;
  LoadCbFuncType load_callback_;
  UnloadCbFuncType unload_callback_;

  ros::NodeHandle nh_;
  ros::ServiceServer load_server_;
  ros::CallbackQueue load_cb_queue_;
  ros::AsyncSpinner load_spinner_;

  temoto_id::IDManager res_id_manager_;
  std::vector<ServerQuery<ServiceType, Owner>> queries_;

  // mutexes
  std::mutex queries_mutex_;
  std::mutex active_server_mutex_;
};
}

#endif
