#ifndef RESOURCE_CLIENT_H
#define RESOURCE_CLIENT_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_client.h"
#include "rmp/resource_manager.h"

namespace rmp
{

template <class Owner>
class ResourceManager;

template<class ServiceMsgType, class Owner>
	class ResourceClient : public BaseResourceClient
{

	public:
		//typedef bool(Owner::*CbFuncType)(typename Service::Request&, typename Service::Response&);

		ResourceClient(std::string name,
				Owner* owner,
				ResourceManager<Owner>& resource_manager
				) : name_(name), owner_(owner), resource_manager_(resource_manager)
		{
			// Setup ROS service client
			service_client_ = nh_.serviceClient<ServiceMsgType>(name_);
		}

		~ResourceClient()
		{
		}


		bool call(ServiceMsgType& msg)
		{
			ROS_INFO("ResourceClient[%s]: Call()", name_);


			ServiceMsgType new_msg;
			bool found = false;
			for(auto act_msg : active_resources_)
			{
				if(act_msg.req == msg.req)
				{
					new_msg = act_msg;
					found = true;
					break;
				}
			}

			if(!found)
			{
				ROS_INFO("ResourceClient[%s]: request not found from existing connections, external call()", name_);
				if(service_client_.call(new_msg))
				{

				}
			}

			active_resources_.push_back(new_msg);
			msg = new_msg;


			return true; 
		}


	private:

	//	std::vector<ResourceEntry<Service>> entries_;

		std::string name_; /// Service name which this client calls.
		Owner* owner_;
		ResourceManager<Owner>& resource_manager_;
		
		ros::ServiceClient service_client_;
		ros::NodeHandle nh_;

		std::vector<ServiceMsgType> active_resources_;


//		typedef std::pair<TemotoID::ID, std::shared_ptr<BaseResourceServer> ServerConnection;
//		std::vector<ServerConnection> internal_bindings_;

//		ResourceManager resource_manager_;

};

}

#endif
