#ifndef BASE_RESOURCE_CLIENT_H
#define BASE_RESOURCE_CLIENT_H

#include <string>
#include "common/temoto_id.h"
#include "rmp/resource_manager.h"

namespace rmp
{


//Forward declatation of ResourceManager
template<class Owner>
class ResourceManager;

template <class Owner>
class BaseResourceClient
{
	public:
		BaseResourceClient(ResourceManager<Owner>& resource_manager) : 
            resource_manager_(resource_manager)
		{
		}
        

		virtual ~BaseResourceClient()
		{
		}

    virtual const std::string& getName() const = 0;
    virtual std::map<temoto_id::ID, std::string> getInternalResources() const = 0;
    virtual const std::map<temoto_id::ID, std::string>
    getInternalResources(temoto_id::ID external_resource_id) const = 0;
    virtual size_t getQueryCount() const = 0;
    virtual void unloadResource(temoto_id::ID resource_id) = 0;
    virtual void setFailedFlag(temoto_id::ID external_resource_id) = 0;
    virtual bool hasFailed(temoto_id::ID internal_resource_id) = 0;
    virtual void unloadResources() = 0;
    virtual bool internalResourceExists(temoto_id::ID) = 0;
    virtual void debug() = 0;

  protected:
		ResourceManager<Owner>& resource_manager_;
        
	private:

};

}



#endif
