#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

//#include "common/temoto_id.h"
#include "common/resource.h"


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

		template<class Service>
		bool addResource(std::string service_name, 
						bool(Owner::*callback)(typename Service::Request&, typename Service::Response&))
		{
			std::shared_ptr<BaseResource> res = std::make_shared<Resource<Service, Owner>>(service_name, callback, owner_);
			resources_.push_back(res);

			return true;
		}

	//	bool isNewRequest(typename Service::Request& req)
	//	{
	//		
	//		return true;
	//	}


	private:

		std::vector<std::shared_ptr<BaseResource>> resources_;
		Owner* owner_;

};  


}   
#endif

