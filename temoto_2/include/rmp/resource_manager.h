#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

//#include "common/temoto_id.h"
#include "rmp/resource_server.h"
#include "rmp/resource_client.h"
#include "rmp/resource_manager_services.h"
#include <string>

// Resource Management Protocol (RMP) for temoto 2

namespace rmp
{

template<class Owner>
class ResourceManager
{
	friend class BaseResourceServer<Owner>;
	public:

		ResourceManager(std::string name, Owner* owner):owner_(owner), name_(name)
		{
			unload_server_ = nh_.advertiseService(name_+"/unload", &ResourceManager<Owner>::unloadCallback, this);
			status_server_ = nh_.advertiseService(name_+"/status", &ResourceManager<Owner>::statusCallback, this);
		}

		~ResourceManager()
		{
			unloadClients();
		}

		template<class ServiceType>
		bool addServer(const std::string& server_name, 
				void(Owner::*load_cb)(typename ServiceType::Request&, typename ServiceType::Response&),
				void(Owner::*unload_cb)(typename ServiceType::Request&, typename ServiceType::Response&))
		{

			if (serverExists(server_name))
			{
				return false;
			}

			typedef std::shared_ptr<BaseResourceServer<Owner>> BaseResPtr;
			BaseResPtr res_srv = std::make_shared<ResourceServer<ServiceType, Owner>> (
					server_name, load_cb, unload_cb, owner_, *this);

			servers_.push_back(res_srv);

			return true;
		}

		bool serverExists(const std::string server_name)
		{
			for (auto& server : servers_)
			{
				if (server->getName() == server_name)
				{
					return true;
				}
				return false;
			}
		}


        const std::string& getName()
        {
            return name_;
        }


		template<class ServiceType>
		bool call(std::string resource_manager_name, std::string server_name, ServiceType& msg)
		{
			std::string prefix = "ResourceManager::call() [" + name_ + "]:";
			ROS_INFO("%s Creating new resource client ...", prefix.c_str());

			using ClientType = ResourceClient<ServiceType, Owner>;
			using ClientPtr = std::shared_ptr<ClientType>;
			ClientPtr client = std::make_shared<ClientType>(resource_manager_name, server_name, owner_, *this);
			bool ret = client->call(msg);

			// Push to clients and convert to BaseResourceClient type
		    clients_.push_back(client);

			// check if this call came from server callback.
			if(active_server_)
			{
				ROS_INFO("%s was called from owners cb (active_server_ is not NULL)", prefix.c_str());
				active_server_->registerInternalClient(client->getName(), msg.response.rmp.resource_id);
			}
			else
			{
				ROS_INFO("%s was not called from owners cb (active_server_ is NULL)", prefix.c_str());
			}

			return ret;
		}

// This method sends error/info message to any client connected to this resource.
		bool sendStatus(temoto_id::ID resource_id, temoto_2::ResourceStatus& status_msg)
		{
            auto s_it = find_if(servers_.begin(), servers_.end(),
                    [&](const std::shared_ptr<BaseResourceServer<Owner>>& server_ptr) -> bool {return server_ptr->internalResourceExists(resource_id);}
                    );
            if(s_it != servers_.end())
            {
                (*s_it)->notifyClients("",status_msg);
            }
            else
            {
                ROS_ERROR("ResourceManager::sendStatus internal resource id was not found from any queries.");
            }
        }




		bool unloadCallback(temoto_2::UnloadResource::Request& req, temoto_2::UnloadResource::Response& res)
		{

            ROS_INFO("[ResourceManager::unloadCallback] %s: Got request: server:%s, id: %ld", name_.c_str(), req.server_name.c_str(), req.resource_id);
            // Find server with requested name
			for (auto& server : servers_)
			{
				if (server->getName() == req.server_name)
				{
                    //try
                    //{
                    server->unloadResource(req, res);
                    ROS_INFO("%s unloadCallback(): Resource %ld unloaded", name_.c_str(), req.resource_id);
                    //}
                    //catch )
                    //{
                    //}
                    //else
                    //{
                    //    ROS_INFO("%s unloadCallback(): ERROR unloading resource with id %ld", name_, req.resource_id);
                    //}
                    break;
				}
			}

            return true;
		}
		

		void unloadClients()
		{
            ROS_INFO("[ResourceManager::unloadClients] [%s] clients:%lu", name_.c_str(), clients_.size());
            for(auto client : clients_)
            {
                std::map<temoto_id::ID, std::string> internal_resources = client->getInternalResources();
               client->unloadResources();
               //TODO: remove all connections in servers
            }
			clients_.clear();
		}


        void unloadClient(std::string client_name, temoto_id::ID resource_id)
        {

            ROS_INFO("[ResourceManager::unloadClient] [%s] name:%s, id:%d", name_.c_str(), client_name.c_str(), resource_id);
            // Go through clients and search for given client by name
            auto client_it = std::find_if(clients_.begin(),clients_.end(),
                   [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool {return client_ptr->getName() == client_name;}
                    );
            if(client_it != clients_.end())
            {
                // found the client, unload resource
                (*client_it)->unloadResource(resource_id);

                // when all resources for this client are closed, destroy this client
                if((*client_it)->getQueryCount() <= 0)
                {
                    clients_.erase(client_it);
                }
            }
            else
            {
                ROS_ERROR("unloadInternalClient: client %s not found", client_name.c_str());
            }
        }


		bool statusCallback(temoto_2::ResourceStatus::Request& req, temoto_2::ResourceStatus::Response& res)
		{
            ROS_INFO("%s: Got status from someone", name_.c_str());
			if (req.status_code == status_codes::FAILED)
			{
				// unload this client, and notify anyone who used it
                
                // Go through clients and locate the one from 
                // which the request arrived
                std::string client_name = req.manager_name+"/"+req.server_name;
                ROS_INFO("Looking for client %s", client_name.c_str());
                auto client_it = std::find_if(clients_.begin(),clients_.end(),
                        [&](const std::shared_ptr<BaseResourceClient<Owner>>& client_ptr) -> bool { ROS_INFO("%s", client_ptr->getName().c_str());
                        return client_ptr->getName() == client_name;}
                        );
                if(client_it != clients_.end())
                {
                    // client found, get all internal calls to this client, and
                    // forward status info to all of them
                    const auto resources = (*client_it)->getInternalResources(req.resource_id);
                    for(const auto& resource : resources)
                    {
                        auto s_it = find_if(servers_.begin(), servers_.end(),
                                [&](const std::shared_ptr<BaseResourceServer<Owner>>& server_ptr) -> bool {return server_ptr->getName() == resource.second;}
                                );
                        if(s_it != servers_.end())
                        {
                            temoto_2::ResourceStatus msg;
                            msg.request = req;
                            msg.response = res;
                            // overwrite id with internal id
                            msg.request.resource_id = resource.first;
                            (*s_it)->notifyClients((*client_it)->getName(), msg);
                        }
                        else
                        {
                            ROS_ERROR("ResourceManager::statusCallback server that was listed in a client query could not be found from servers_");
                        }
                    }
                }
                else
                {
                    ROS_INFO("%s: statusCallback failed to find the client", name_.c_str());
                }
			}
				return true;
        }


        temoto_id::ID generateInternalID()
        {
            return internal_id_manager_.generateID();
        }


        void notifyServer(temoto_2::ResourceStatus& msg)
        {
            ROS_INFO("Sending notification to '%s', id:'%ld'",
                   msg.request.server_name, msg.request.resource_id);
        }


        const std::string getActiveServerName() const 
        {
            if(active_server_)
            {
                return active_server_->getName();
            }
            return std::string("");
        }
        

	private:


		bool setActiveServer(BaseResourceServer<Owner>* active_server)
		{
			for(auto& server_shared_ptr : servers_)
			{
				if(server_shared_ptr.get() == active_server)
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

		ros::NodeHandle nh_;
		ros::ServiceServer unload_server_;
		ros::ServiceServer status_server_;
};  


} // namespace rmp 

#endif
