#ifndef BASE_RESOURCE_CLIENT_H
#define BASE_RESOURCE_CLIENT_H

#include <string>
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "rmp/resource_manager.h"
#include "rmp/client_query.h"

namespace rmp
{


//Forward declatation of ResourceManager
template<class Owner>
class ResourceManager;

template <class Owner>
class BaseResourceClient : public BaseSubsystem
{
	public:
    BaseResourceClient(ResourceManager<Owner>& resource_manager)
      : BaseSubsystem(resource_manager), resource_manager_(resource_manager)
    {
    }
        
		virtual ~BaseResourceClient()
		{
		}

    virtual const std::string& getName() const = 0;
    virtual std::map<temoto_id::ID, FailureBehavior> getInternalResources() const = 0;
    virtual const std::map<temoto_id::ID, FailureBehavior>
    getInternalResources(temoto_id::ID external_resource_id) = 0;
    virtual size_t getQueryCount() const = 0;
    virtual void unloadResource(temoto_id::ID resource_id) = 0;
    virtual void setFailedFlag(temoto_id::ID external_resource_id) = 0;
    virtual bool hasFailed(temoto_id::ID internal_resource_id) = 0;
    virtual void unloadResources() = 0;
    virtual bool internalResourceExists(temoto_id::ID) = 0;
    virtual std::string toString() = 0;

  protected:
		ResourceManager<Owner>& resource_manager_;
        
	private:

};

template <class Owner>
using BaseResourceClientPtr = std::shared_ptr<BaseResourceClient<Owner>>;
}



#endif
