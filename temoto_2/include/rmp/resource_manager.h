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
			typedef std::shared_ptr<BaseResourceServer> BaseResPtr;
			BaseResPtr res_srv = std::make_shared<ResourceServer< LoadService, UnloadService, Owner > > (
					server_name, load_cb, unload_cb, owner_, *this);

			servers_.push_back(res_srv);

			return true;
		}


		temoto_id::ID checkClientID(temoto_id::ID client_id)
		{
			return ext_client_id_manager_.checkID(client_id);
		}

		
//		template<class ServiceMsgType>
//		bool call(std::string client_name, std::string server_name, ServiceMsgType& msg)
//		{
//			// check if client with the requested name exists.
//			if(true)
//			{
//
//			}
//			else
//			{
//				std::shared_ptr<BaseResourceClient> res_client = std::make_shared<ResourceClient<ServiceMsgType, Owner>>(service_name, owner_);
//				clients_.push_back(res_client);
//			}
////
//
//			return true;
//		}

	//	bool isNewRequest(typename Service::Request& req)
	//	{
	//		
	//		return true;
	//	}


	private:

		std::vector<std::shared_ptr<BaseResourceServer>> servers_;
		std::vector<std::shared_ptr<BaseResourceClient>> clients_;
		Owner* owner_;
		temoto_id::IDManager ext_client_id_manager_;

};  


}   
#endif

