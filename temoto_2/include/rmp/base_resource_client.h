#ifndef BASE_RESOURCE_CLIENT_H
#define BASE_RESOURCE_CLIENT_H

//#include "rmp/resource_entry.h"
#include "common/temoto_id.h"
#include <string>

namespace rmp
{

class BaseResourceClient
{
	public:
		BaseResourceClient()
		{
		}
		virtual ~BaseResourceClient()
		{
		}

        virtual size_t getConnectionCount() const = 0;
        virtual void unloadResource(temoto_id::ID resource_id) const = 0;
        virtual const std::string& getName() const = 0;
        virtual void unloadInternalClient(temoto_id::ID resource_id) = 0;


	private:

};

}



#endif
