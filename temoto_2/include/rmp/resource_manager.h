#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

//#include "common/temoto_id.h"
#include "rmp/resource_server.h"
#include "rmp/resource_client.h"
#include <string>

// Resource Management Protocol (RMP) for temoto 2

namespace rmp
{

template<class Owner>
class ResourceManager
{
	friend class BaseResourceServer<Owner>;
	public:

		ResourceManager(Owner* owner):owner_(owner)
		{
		}

		~ResourceManager()
		{
		}

		template<class LoadService, class UnloadService>
		bool addServer(std::string server_name, 
				bool(Owner::*load_cb)(typename LoadService::Request&, typename LoadService::Response&),
				bool(Owner::*unload_cb)(typename UnloadService::Request&, typename UnloadService::Response&))
		{

			if (serverExists(server_name))
			{
				return false;
			}

			typedef std::shared_ptr<BaseResourceServer<Owner>> BaseResPtr;
			BaseResPtr res_srv = std::make_shared<ResourceServer<LoadService, UnloadService, Owner>> (
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
		};


		temoto_id::ID registerExternalClient(temoto_id::ID client_id)
		{
			return ext_client_id_manager_.checkID(client_id);
		}




		template<class ServiceMsgType>
		bool call(std::string client_name, ServiceMsgType& msg)
		{
			std::shared_ptr<BaseResourceClient> res_client = std::make_shared<ResourceClient<ServiceMsgType, Owner>>(client_name, owner_);
			clients_.push_back(res_client);


			// check if this call came from server callback.
			if(active_server_)
			{
				active_server_->registerInternalClient(client_name, msg.res.resource_id);
			}
			else
			{
				ROS_INFO("not from callback, owner has to take care of shutting down client connection");
			}

			return true;
		}

// This method sends error/info message to any client connected to this resource.
		bool sendStatus(temoto_id::ID resource_id, temoto_2::ResourceStatus& status_msg)
		{
//TODO: implement me
		};



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
		};

		std::vector<std::shared_ptr<BaseResourceServer<Owner>>> servers_;
		std::vector<std::shared_ptr<BaseResourceClient>> clients_;
		Owner* owner_;
		temoto_id::IDManager ext_client_id_manager_;
		std::shared_ptr<BaseResourceServer<Owner>> active_server_;
};  


}   
#endif

