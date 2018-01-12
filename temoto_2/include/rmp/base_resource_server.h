#ifndef BASE_RESOURCE_SERVER_H
#define BASE_RESOURCE_SERVER_H

#include <string>
#include "temoto_error/temoto_error.h"
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "rmp/resource_manager.h"
#include <vector>
#include <utility>

namespace rmp
{

//Forward declatation of ResourceManager
template<class Owner>
class ResourceManager;

template<class Owner>
class BaseResourceServer : public BaseSubsystem
{
	public:
//		BaseResourceServer()
//		{
//			name_ = "default_server_name";
//			resource_manager_ = NULL;
//		}

		BaseResourceServer(const std::string name, ResourceManager<Owner>& resource_manager) :
      BaseSubsystem (resource_manager),
			name_(name),
			resource_manager_(resource_manager)
		{
		}
		virtual ~BaseResourceServer()
		{
		}

    virtual void setFailedFlag(temoto_id::ID internal_resource_id, error::ErrorStack& error_stack) = 0;

    const std::string& getName()
		{
			return name_;
		};

		virtual void linkInternalResource(temoto_id::ID resource_id) = 0;
		virtual void unlinkInternalResource(temoto_id::ID resource_id) = 0;
		virtual bool isLinkedTo(temoto_id::ID resource_id) const = 0;
		virtual bool hasInternalResource(temoto_id::ID resource_id) const = 0;
		virtual bool hasExternalResource(temoto_id::ID resource_id) const = 0;
		virtual void unloadResource(temoto_2::UnloadResource::Request& req, temoto_2::UnloadResource::Response& res) = 0;
    virtual std::vector<std::pair<temoto_id::ID, std::string>>
    getExternalResourcesByInternalId(temoto_id::ID internal_resource_id) = 0;
    virtual std::vector<std::pair<temoto_id::ID, std::string>>
    getExternalResourcesByExternalId(temoto_id::ID external_resource_id) = 0;

  protected:

		void activateServer()
		{
			resource_manager_.setActiveServer(this);
		};

		void deactivateServer()
		{
			resource_manager_.setActiveServer(NULL);
		};

		ResourceManager<Owner>& resource_manager_;
		std::string name_;


	private:
};

}



#endif
