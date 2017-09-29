#ifndef BASE_RESOURCE_SERVER_H
#define BASE_RESOURCE_SERVER_H

//#include "rmp/resource_entry.h"
#include <string>
#include "common/temoto_id.h"

namespace rmp
{

class BaseResourceServer
{
	public:
		BaseResourceServer()
		{
		}
		virtual ~BaseResourceServer()
		{
		}
		//template <class Service>
		//bool addResourceEntry(ResourceEntry<Service> resource)=0;

		const std::string& getName()
		{
			return name_;
		};

		virtual void registerInternalClient(std::string client_name, temoto_id::ID resource_id){};

	protected:
		std::string name_;

	private:
};

}



#endif
