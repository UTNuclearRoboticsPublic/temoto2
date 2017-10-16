#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

//#include "common/temoto_id.h"
#include "rmp/resource_server.h"
#include "rmp/resource_client.h"
#include "rmp/resource_manager_services.h"
#include <string>
#include <memory>  // dynamic_pointer_cast

// Resource Management Protocol (RMP) for temoto 2

namespace rmp
{
template <class Owner>
class ResourceManager
{
  friend class BaseResourceServer<Owner>;

public:
  ResourceManager(std::string name, Owner* owner)
    : owner_(owner), name_(name), status_callback_(NULL)
  {
    unload_server_ =
        nh_.advertiseService(name_ + "/unload", &ResourceManager<Owner>::unloadCallback, this);
    status_server_ =
        nh_.advertiseService(name_ + "/status", &ResourceManager<Owner>::statusCallback, this);
    last_generated_id = temoto_id::UNASSIGNED_ID;
  }

  ~ResourceManager()
  {
    unloadClients();
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
  bool call(std::string resource_manager_name, std::string server_name, ServiceType& msg)
  {
    std::string prefix = "ResourceManager::call() [" + name_ + "]:";
    using ClientType = ResourceClient<ServiceType, Owner>;
    using ClientPtr = std::shared_ptr<ClientType>;
    using BaseClientType = BaseResourceClient<Owner>;
    using BaseClientPtr = std::shared_ptr<BaseClientType>;

    ClientPtr client_ptr = NULL;

    // check if this client already exists
    std::string client_name = resource_manager_name + "/" + server_name;
    typename std::vector<BaseClientPtr>::iterator client_it =
        std::find_if(clients_.begin(), clients_.end(),
                     [&](const BaseClientPtr& c) -> bool { return c->getName() == client_name; });
    if (client_it == clients_.end())
    {
      ROS_INFO("%s Creating new resource client ...", prefix.c_str());

      client_ptr = std::make_shared<ClientType>(resource_manager_name, server_name, owner_, *this);
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
        ROS_ERROR("%s Dynamic Cast failed, someone is misusing RMP", prefix.c_str());
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
      ROS_INFO("%s called from servers callback, registering internal client", prefix.c_str());
      active_server_->registerInternalClient(client_ptr->getName(), msg.response.rmp.resource_id);
    }

    return ret;
  }

  // This method sends error/info message to any client connected to this resource.
  void sendStatus(temoto_id::ID resource_id, temoto_2::ResourceStatus& status_msg)
  {
    auto s_it = find_if(servers_.begin(), servers_.end(),
                        [&](const std::shared_ptr<BaseResourceServer<Owner>>& server_ptr) -> bool {
                          return server_ptr->internalResourceExists(resource_id);
                        });
    if (s_it != servers_.end())
    {
      (*s_it)->notifyClients(status_msg);
    }
    else
    {
      ROS_ERROR("ResourceManager::sendStatus internal resource id was not found from any queries.");
    }
  }

  bool unloadCallback(temoto_2::UnloadResource::Request& req,
                      temoto_2::UnloadResource::Response& res)
  {
    ROS_INFO("[ResourceManager::unloadCallback] %s: Unload request to server:%s, ext id: %ld",
             name_.c_str(), req.server_name.c_str(), req.resource_id);
    // Find server with requested name
    for (auto& server : servers_)
    {
      if (server->getName() == req.server_name)
      {
        // try
        //{
        server->unloadResource(req, res);
        ROS_INFO("%s unloadCallback(): Resource %ld unloaded", name_.c_str(), req.resource_id);
        //}
        // catch )
        //{
        //}
        // else
        //{
        //    ROS_INFO("%s unloadCallback(): ERROR unloading resource with id %ld", name_,
        //    req.resource_id);
        //}
        break;
      }
    }

    return true;
  }

  void unloadClients()
  {
    ROS_INFO("[ResourceManager::unloadClients] [%s] clients:%lu", name_.c_str(), clients_.size());
    for (auto client : clients_)
    {
      std::map<temoto_id::ID, std::string> internal_resources = client->getInternalResources();
      client->unloadResources();
      // TODO: remove all connections in servers
    }
    clients_.clear();
  }

  // For unloading owners resources
  void unloadResource(temoto_id::ID resource_id)
  {
    unloadClient("", resource_id);
  }

  // Wrapper for unloading resource clients
  void unloadClient(std::string client_name, temoto_id::ID resource_id)
  {
    ROS_INFO("[ResourceManager::unloadClient] [%s] name:%s, id:%d", name_.c_str(),
             client_name.c_str(), resource_id);
    // Go through clients and search for given client by name
    auto client_it =
        std::find_if(clients_.begin(), clients_.end(),
                     [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {
                       return client_ptr->getName() == client_name;
                     });
    if (client_it != clients_.end())
    {
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
      ROS_WARN("ResourceMananager::unloadInternalClient[%s]: client '%s' not found. Already "
               "removed?",
               name_.c_str(), client_name.c_str());
    }
  }

  bool statusCallback(temoto_2::ResourceStatus::Request& req,
                      temoto_2::ResourceStatus::Response& res)
  {
    std::string prefix = "ResourceManager::statusCallback [" + name_ + "]:";
    ROS_INFO("%s Got status: ", prefix.c_str());
    ROS_INFO_STREAM(req);


    if (req.status_code == status_codes::FAILED)
    {
      // unload this client, and notify anyone who used it
      ROS_INFO("START DEBUGGING CLIENTS");
      for (auto& client : clients_)
      {
        client->debug();
      }
      ROS_INFO("END DEBUGGING CLIENTS");
      // Go through clients and locate the one from
      // which the request arrived
      std::string client_name = req.manager_name + "/" + req.server_name;
      ROS_INFO("%s code==failed -> Looking for client %s", prefix.c_str(), client_name.c_str());
      auto client_it =
          std::find_if(clients_.begin(), clients_.end(),
                       [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {
                         ROS_INFO("comparing client: %s", client_ptr->getName().c_str());
                         return client_ptr->getName() == client_name;
                       });
      if (client_it != clients_.end())
      {
        // client found, get all internal calls to this client, and
        // forward status info to all of them
        const auto resources = (*client_it)->getInternalResources(req.resource_id);
        // ROS_INFO("RESOURCES: %lu", resources.size());
        for (const auto& resource : resources)
        {
          temoto_2::ResourceStatus srv;
          srv.request = req;
          srv.response = res;
          // overwrite id with internal id
          srv.request.resource_id = resource.first;
          // call owners status callback if registered
          if (status_callback_)
          {
            (owner_->*status_callback_)(srv);
          }

          auto s_it =
              find_if(servers_.begin(), servers_.end(),
                      [&](const std::shared_ptr<BaseResourceServer<Owner>>& server_ptr) -> bool {
                        return server_ptr->getName() == resource.second;
                      });
          if (s_it != servers_.end())
          {
            // notify all connected clients in the server
            (*s_it)->notifyClients(srv);
          }
          else
          {
            //   ROS_ERROR("ResourceManager::statusCallback server that was listed in a client query
            //   "
            //            "could not be found from servers_");
          }
        }
      }
      else
      {
        // ROS_INFO("%s: statusCallback failed to find the client", name_.c_str());
      }
    }

    return true;
  }

  temoto_id::ID generateInternalID()
  {
    last_generated_id = internal_id_manager_.generateID();
    return last_generated_id;
  }

  void notifyServer(temoto_2::ResourceStatus& msg)
  {
    ROS_INFO("Sending notification to '%s', id:'%ld'", msg.request.server_name,
             msg.request.resource_id);
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

  std::vector<std::shared_ptr<BaseResourceServer<Owner>>> servers_;
  std::vector<std::shared_ptr<BaseResourceClient<Owner>>> clients_;
  std::string name_;
  Owner* owner_;
  temoto_id::IDManager internal_id_manager_;
  std::shared_ptr<BaseResourceServer<Owner>> active_server_;
  temoto_id::ID last_generated_id;
  void (Owner::*status_callback_)(temoto_2::ResourceStatus& srv);

  ros::NodeHandle nh_;
  ros::ServiceServer unload_server_;
  ros::ServiceServer status_server_;
};

}  // namespace rmp

#endif
