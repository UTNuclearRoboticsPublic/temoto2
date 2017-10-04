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
			using ClientType = ResourceClient<ServiceType, Owner>;
			using ClientPtr = std::shared_ptr<ClientType>;
			ClientPtr client = std::make_shared<ClientType>(resource_manager_name, server_name, owner_);
			client->call(msg);
			// Push to clients and convert to BaseResourceClient type
		    clients_.push_back(client);

			// check if this call came from server callback.
			if(active_server_)
			{
				active_server_->registerInternalClient(client->getName(), msg.response.resource_id);
				ROS_INFO("ResourceClient::call() from callback, client and server linked");
			}
			else
			{
				ROS_INFO("ResourceClient::call() not from callback");
			}

			return true;
		}

// This method sends error/info message to any client connected to this resource.
		bool sendStatus(temoto_id::ID resource_id, temoto_2::ResourceStatus& status_msg)
		{
//TODO: implement me
		}




		bool unloadCallback(temoto_2::UnloadResource::Request& req, temoto_2::UnloadResource::Response& res)
		{
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


        void unloadClient(std::string client_name, temoto_id::ID resource_id)
        {
            // Go through clients and search for given client by name
            auto client_it = std::find_if(clients_.begin(),clients_.end(),
                   [&](const std::shared_ptr<BaseResourceClient>& client_ptr) -> bool {return client_ptr->getName() == client_name;}
                    );
            if(client_it != clients_.end())
            {
                // found the client, unload resource
                (*client_it)->unloadResource(resource_id);

                // when all resources for this client are closed, destroy this client
                if((*client_it)->getConnectionCount() <= 0)
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
		std::vector<std::shared_ptr<BaseResourceClient>> clients_;
        std::string name_;
		Owner* owner_;
//		temoto_id::IDManager ext_client_id_manager_;
		std::shared_ptr<BaseResourceServer<Owner>> active_server_;

		ros::NodeHandle nh_;
		ros::ServiceServer unload_server_;
		ros::ServiceServer status_server_;
};  


}   
#endif

