#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "rmp/resource_server.h"
#include "rmp/resource_client.h"
#include "rmp/resource_manager_services.h"
#include "rmp/log_macros.h"
#include "ros/callback_queue.h"
#include <string>
#include <memory>  // dynamic_pointer_cast
#include <mutex>

// Resource Management Protocol (RMP) for temoto 2

namespace rmp
{
template <class Owner>
class ResourceManager
{
  friend class BaseResourceServer<Owner>;

public:
  ResourceManager(std::string name, Owner* owner)
    : owner_(owner)
    , name_(name)
    , status_callback_(NULL)
    , status_spinner_(1, &status_cb_queue_)
    , unload_spinner_(1, &unload_cb_queue_)
  {
    log_class_ = "rmp/Manager";
    log_subsys_ = name;
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
    RMP_DEBUG("%s Resource manager is starting.", prefix.c_str());

    // set up status callback with separately threaded queue
    //std::string status_srv_name = srv_name::PREFIX + "/" + name_ + "/status";
    std::string status_srv_name = name_ + "/status";
    ros::AdvertiseServiceOptions status_service_opts =
        ros::AdvertiseServiceOptions::create<temoto_2::ResourceStatus>(
            status_srv_name, boost::bind(&ResourceManager<Owner>::statusCallback, this, _1, _2),
            ros::VoidPtr(), &this->status_cb_queue_);
    status_server_ = nh_.advertiseService(status_service_opts);

    // set up unload callback with separately threaded queue
//    std::string unload_srv_name = srv_name::PREFIX + "/" + name_ + "/unload";
    std::string unload_srv_name = name_ + "/unload";
    ros::AdvertiseServiceOptions unload_service_opts =
        ros::AdvertiseServiceOptions::create<temoto_2::UnloadResource>(
            unload_srv_name, boost::bind(&ResourceManager<Owner>::unloadCallback, this, _1, _2),
            ros::VoidPtr(), &this->unload_cb_queue_);
    unload_server_ = nh_.advertiseService(unload_service_opts);

    last_generated_id = temoto_id::UNASSIGNED_ID;

    // start separate threaded spinners for our callback queues
    status_spinner_.start();
    unload_spinner_.start();
//    RMP_DEBUG("%s Resource manager started.", prefix.c_str());
  }

  ~ResourceManager()
  {
    unloadClients();
    unload_spinner_.stop();
    status_spinner_.stop();
  }

  template <class ServiceType>
  bool addServer(const std::string& server_name,
                 void (Owner::*load_cb)(typename ServiceType::Request&,
                                        typename ServiceType::Response&),
                 void (Owner::*unload_cb)(typename ServiceType::Request&,
                                          typename ServiceType::Response&))
  {
    if (serverExists(server_name))
    {
      return false;
    }

    typedef std::shared_ptr<BaseResourceServer<Owner>> BaseResPtr;
    BaseResPtr res_srv = std::make_shared<ResourceServer<ServiceType, Owner>>(
        server_name, load_cb, unload_cb, owner_, *this);

    servers_.push_back(res_srv);

    return true;
  }

  void registerStatusCb(void (Owner::*status_cb)(temoto_2::ResourceStatus&))
  {
    status_callback_ = status_cb;
  }

  bool serverExists(const std::string server_name)
  {
    for (auto& server : servers_)
    {
      if (server->getName() == server_name)
      {
        return true;
      }
    }
    return false;
  }

  const std::string& getName()
  {
    return name_;
  }

  template <class ServiceType>
  bool call(std::string resource_manager_name, std::string server_name, ServiceType& msg, std::string temoto_namespace = ::common::getTemotoNamespace())
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");
    using ClientType = ResourceClient<ServiceType, Owner>;
    using ClientPtr = std::shared_ptr<ClientType>;
    using BaseClientType = BaseResourceClient<Owner>;
    using BaseClientPtr = std::shared_ptr<BaseClientType>;

    ClientPtr client_ptr = NULL;

    waitForLock(clients_mutex_);

    // check if this client already exists
    std::string client_name = '/'+temoto_namespace + "/" + resource_manager_name + "/" + server_name;
    typename std::vector<BaseClientPtr>::iterator client_it =
        std::find_if(clients_.begin(), clients_.end(),
                     [&](const BaseClientPtr& c) -> bool { return c->getName() == client_name; });
    if (client_it == clients_.end())
    {
      RMP_DEBUG("%s Creating new resource client. %s", prefix.c_str(), log_subsys_.c_str());

      client_ptr = std::make_shared<ClientType>(temoto_namespace, resource_manager_name,
                                                server_name, owner_, *this);
      // Push to clients and convert to BaseResourceClient type
      clients_.push_back(client_ptr);
      client_it = std::prev(clients_.end());  // set iterator to the client pointer we just added
    }
    else
    {
      client_ptr = std::dynamic_pointer_cast<ClientType>(*client_it);
      if (!client_ptr)
      {
        // cast failed
        RMP_ERROR("%s Dynamic Cast failed, someone is misusing RMP",
                        prefix.c_str());
        return false;
      }
    }

    if (!active_server_)
    {
      // generate new if for owner as the call was not initiated from any servers callback
      msg.response.rmp.resource_id = generateInternalID();
    }
    else
    {
      // called from servers callback. use the same internal ID for client to make then binding
      msg.response.rmp.resource_id = last_generated_id;
    }

    // make the call to server
    bool ret = client_ptr->call(msg);

    // if call was sucessful and the call was initiated from server callback,
    // register the client in server
    if (ret && active_server_)  // @TODO: go from bool ret to try-catch
    {
      RMP_DEBUG("%s called from servers callback, registering internal client",
                      prefix.c_str());
      active_server_->registerInternalResource(client_ptr->getName(), msg.response.rmp.resource_id);
    }

    clients_mutex_.unlock();

    return ret;
  }

  bool unloadCallback(temoto_2::UnloadResource::Request& req,
                      temoto_2::UnloadResource::Response& res)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    RMP_DEBUG("%s Unload request to server: '%s', ext id: %ld", prefix.c_str(),
                    req.server_name.c_str(), req.resource_id);
    // Find server with requested name
    waitForLock(servers_mutex_);
    for (auto& server : servers_)
    {
      if (server->getName() == req.server_name)
      {
        server->unloadResource(req, res);
        RMP_DEBUG("%s Resource %ld unloaded", prefix.c_str(), req.resource_id);
        break;
      }
    }
    servers_mutex_.unlock();

    return true;
  }

  void unloadClients()
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    waitForLock(clients_mutex_);
    RMP_DEBUG("%s number of clients:%lu", prefix.c_str(), clients_.size());
    for (auto client : clients_)
    {
      std::map<temoto_id::ID, std::string> internal_resources = client->getInternalResources();
      client->unloadResources();
      // TODO: remove all connections in servers
    }
    clients_.clear();
    clients_mutex_.unlock();
  }

  // Wrapper for unloading resource clients
  void unloadClientResource(temoto_id::ID resource_id)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    waitForLock(clients_mutex_);
    RMP_DEBUG("%s id:%d", prefix.c_str(), resource_id);
    // Go through clients and search for given client by name
    auto client_it =
        std::find_if(clients_.begin(), clients_.end(),
                     [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {
                       return client_ptr->internalResourceExists(resource_id);
                     });
    if (client_it != clients_.end())
    {
      RMP_DEBUG("%s id:%d found, executing client.unload()", prefix.c_str(), resource_id);
      // found the client, unload resource
      (*client_it)->unloadResource(resource_id);

      // when all resources for this client are closed, destroy this client
      if ((*client_it)->getQueryCount() <= 0)
      {
        clients_.erase(client_it);
      }
    }
    else
    {
      RMP_WARN("%s Internal id '%d' not found. Already removed?", prefix.c_str(),
                     resource_id);
    }
    clients_mutex_.unlock();
  }

  // This method sends error/info message to any client connected to this resource.
  void sendStatus(temoto_2::ResourceStatus& srv)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

    // For storing service and status topic temporarily.
    struct ResourceInfo
    {
      std::string status_topic;
      temoto_2::ResourceStatus srv;
    };
    std::vector<ResourceInfo> infos;

    waitForLock(servers_mutex_);
    for (const auto& server : servers_)
    {
      auto ext_resources = server->getExternalResources(srv.request.resource_id);
      ResourceInfo info;
      info.srv = srv;  // initialize with given values
      info.srv.request.temoto_namespace = ::common::getTemotoNamespace();
      info.srv.request.manager_name = name_;
      info.srv.request.server_name = server->getName();
      for (const auto& ext_resource : ext_resources)
      {
        // ext_resource.first ==> external resource id
        // ext_resource.second ==> status topic
        info.srv.request.resource_id = ext_resource.first;
        info.status_topic = ext_resource.second;
        infos.push_back(info);

      }
      
     // TODO: the following is temporary hack
     // set FAILED flag for resource queries. The flag is checked when new query arrives to the server
     server->setFailedFlag(srv.request.resource_id);
    }
    servers_mutex_.unlock();

    // forward status info to whoever is related with the given resource
    for (auto& info : infos)
    {
      ros::ServiceClient service_client =
          nh_.serviceClient<temoto_2::ResourceStatus>(info.status_topic);
      RMP_DEBUG("%s Sending ResourceStatus to %s", prefix.c_str(),
                      info.status_topic.c_str());
      if (service_client.call(info.srv))
      {
        RMP_DEBUG("%s ResourceStatus sucessfully sent to %s", prefix.c_str(),
                        info.status_topic.c_str());
      }
      else
      {
        RMP_ERROR("%s Failed to send ResourceStatus to %s", prefix.c_str(),
                        info.status_topic.c_str());
      }
    }

    if (infos.empty())
    {
      RMP_ERROR("%s Internal resource id was not found from any queries.",
                      prefix.c_str());
    }
  }

// This method returns true when statusCallback() has set the FAILED flag to the given resource
  bool hasFailed(temoto_id::ID resource_id)
  {
    waitForLock(clients_mutex_);
    auto client_it =
        std::find_if(clients_.begin(), clients_.end(),
                     [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {
                       return client_ptr->hasFailed(resource_id);
                     });
    bool has_failed =  client_it != clients_.end();
    clients_mutex_.unlock();
    return has_failed;
  }

  bool statusCallback(temoto_2::ResourceStatus::Request& req,
                      temoto_2::ResourceStatus::Response& res)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    RMP_DEBUG("%s Got status request: ", prefix.c_str());
    ROS_INFO_STREAM(req);

    // based on incoming external id, find the assigned internal ids
    // and for each internal id, find the external ids and do forwarding

    if (req.status_code == status_codes::FAILED)
    {
      // reqister failed resource
      
      //      ROS_INFO("START DEBUGGING CLIENTS");
      //      for (auto& client : clients_)
      //      {
      //        client->debug();
      //      }
      //      ROS_INFO("END DEBUGGING CLIENTS");

      // Go through clients and locate the one from
      // which the request arrived
      std::string client_name =
          "/" + req.temoto_namespace + "/" + req.manager_name + "/" + req.server_name;
      RMP_DEBUG("%s got info that resource has failed, looking for %s",
                      prefix.c_str(), client_name.c_str());

      waitForLock(clients_mutex_);
      auto client_it =
          std::find_if(clients_.begin(), clients_.end(),
                       [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {
                         // ROS_INFO("comparing client: %s", client_ptr->getName().c_str());
                         return client_ptr->getName() == client_name;
                       });
      // internal ids that are related to the incoming external id from client side
      std::vector<temoto_id::ID> int_ids;
      if (client_it != clients_.end())
      {
        const auto int_resources = (*client_it)->getInternalResources(req.resource_id);
        for (const auto& int_resource : int_resources)
        {
          // int_resource.first ==> internal resource id
          // int_resource.second ==> server name
          int_ids.emplace_back(int_resource.first);
        }
        (*client_it)->setFailedFlag(req.resource_id);
      }
      else
      {
        RMP_ERROR("%s Could not find client that has failed.", prefix.c_str());
      }
      clients_mutex_.unlock();

      //
      temoto_2::ResourceStatus srv;
      srv.request = req;
      srv.response = res;
      for (auto int_resource_id : int_ids)
      {
        // for each internal resource, execute owner's callback
        srv.request.resource_id = int_resource_id;
        if (status_callback_)
        {
          (owner_->*status_callback_)(srv);
        }

        // forward status info to whoever is related with the given internal resource
        sendStatus(srv);
      }
    }  // if code == FAILED
    return true;
  }

  std::vector<std::pair<temoto_id::ID, std::string>>
  getServerExtResources(temoto_id::ID internal_resource_id, const std::string& server_name)
  {
    waitForLock(servers_mutex_);
    std::vector<std::pair<temoto_id::ID, std::string>> ext_resources;
    auto s_it = find_if(servers_.begin(), servers_.end(),
                        [&](const std::shared_ptr<BaseResourceServer<Owner>>& server_ptr) -> bool {
                          return server_ptr->getName() == server_name;
                        });
    if (s_it != servers_.end())
    {
      ext_resources = (*s_it)->getExternalResources(internal_resource_id);
    }
    servers_mutex_.unlock();
    return ext_resources;
  }

  temoto_id::ID generateInternalID()
  {
    std::lock_guard<std::mutex> lock(id_manager_mutex_);
    last_generated_id = internal_id_manager_.generateID();
    return last_generated_id;
  }

  const std::string getActiveServerName() const
  {
    if (active_server_)
    {
      return active_server_->getName();
    }
    return std::string("");
  }

private:
  bool setActiveServer(BaseResourceServer<Owner>* active_server)
  {
    std::lock_guard<std::mutex> lock(active_server_mutex_);
    for (auto& server_shared_ptr : servers_)
    {
      if (server_shared_ptr.get() == active_server)
      {
        active_server_ = server_shared_ptr;
        return true;
      }
    }
    return false;
  }

  void waitForLock(std::mutex& m)
  {
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    while (!m.try_lock())
    {
      RMP_DEBUG("%s Waiting for lock()", prefix.c_str());
      ros::Duration(0.2).sleep();  // sleep for few ms
    }
    // RMP_DEBUG("%s Obtained lock()", prefix.c_str());
  }

  std::vector<std::shared_ptr<BaseResourceServer<Owner>>> servers_;
  std::vector<std::shared_ptr<BaseResourceClient<Owner>>> clients_;
  std::string name_;
  Owner* owner_;
  temoto_id::IDManager internal_id_manager_;
  std::shared_ptr<BaseResourceServer<Owner>> active_server_;
  temoto_id::ID last_generated_id;
  void (Owner::*status_callback_)(temoto_2::ResourceStatus&);
  std::set<temoto_id::ID> failed_resources_;

  ros::AsyncSpinner status_spinner_;
  ros::AsyncSpinner unload_spinner_;
  ros::CallbackQueue status_cb_queue_;
  ros::CallbackQueue unload_cb_queue_;
  ros::NodeHandle nh_;
  ros::ServiceServer unload_server_;
  ros::ServiceServer status_server_;

  // thread safe locks;
  std::mutex id_manager_mutex_;
  std::mutex servers_mutex_;
  std::mutex clients_mutex_;
  std::mutex active_server_mutex_;
  std::mutex failed_resource_mutex_;

  // log prefixes
  std::string log_subsys_, log_class_;
};

}  // namespace rmp

#endif
