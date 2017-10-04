#ifndef RESOURCE_CLIENT_H
#define RESOURCE_CLIENT_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_client.h"
//#include "rmp/resource_manager.h"
#include <string>
#include <set>

namespace rmp
{

//template <class Owner>
//class ResourceManager;

template<class ServiceType, class Owner>
	class ResourceClient : public BaseResourceClient
{

	public:
		//typedef bool(Owner::*CbFuncType)(typename Service::Request&, typename Service::Response&);

		ResourceClient(std::string resource_manager_name, std::string server_name,
				Owner* owner 
				) : resource_manager_name_(resource_manager_name), server_name_(server_name), 
                    owner_(owner)
		{
            name_ = resource_manager_name + "/" + server_name;
            
			// Setup ROS service clients for loading and unloading the resource
            std::string unload_name = resource_manager_name+"/unload";
			service_client_ = nh_.serviceClient<ServiceType>(name_);
			service_client_unload_ = nh_.serviceClient<temoto_2::UnloadResource>(unload_name);
		}

		~ResourceClient()
		{
            // Send unload service request to all remaining active connections
            for(const auto& msg : active_resources_)
            {
                temoto_2::UnloadResource unload_msg;
                unload_msg.request.server_name = server_name_;
                unload_msg.request.resource_id = msg.response.resource_id;
                service_client_unload_.call(unload_msg);
            }
		}


		bool call(ServiceType& msg)
		{
			ROS_INFO("ResourceClient[%s]: Call()", name_.c_str());


			// search for given request from active resources 
			auto msg_it = std::find_if(active_resources_.begin(), active_resources_.end(), 
					[&](const ServiceType& msg) -> bool {return msg.request == msg.request;}
                    ); 
            if(msg_it == active_resources_.end())
            {
                // New request
				ROS_INFO("ResourceClient[%s]: new request, performing external call()", name_.c_str());
				if(service_client_.call(msg))
				{
                    active_resources_.push_back(msg);
					auto msg_it  = std::prev(active_resources_.end()); // set iterator to the added element
				}
                else
                {
                   ROS_ERROR("ResourceClient[%s]: service call failed", name_.c_str()); 
                   return false;
                }
			}

            // Generate id for new resource and add it to the map
            temoto_id::ID generated_id = id_manager_.generateID();
            ids.emplace(generated_id, msg_it->response.resource_id);

            // Update id in response part
            msg.response.resource_id = generated_id;

			return true; 
		}

        
        void unloadResource(temoto_id::ID resource_id)
        {
			// search for given resource id to unload 
			auto found_msg = std::find_if(active_resources_.begin(), active_resources_.end(), 
					[&](const ServiceType& msg) -> bool {return msg.response.resource_id == resource_id;}
                    ); 
            if(found_msg != active_resources_.end())
            {
                active_resources_.erase(found_msg);
            }
        }

        size_t getConnectionCount() const
        {
            return active_resources_.size();
        }

        const std::string& getName() const {return name_;} 


	private:

		std::string name_; /// The unique name of a resource client.
		std::string server_name_; /// Server name which this client calls.
		std::string resource_manager_name_; /// Name of resource manager where the server is located.
		Owner* owner_;
		//ResourceManager<Owner>& resource_manager_;
		temoto_id::IDManager id_manager_;
		
		ros::ServiceClient service_client_;
		ros::ServiceClient service_client_unload_;
		ros::NodeHandle nh_;

		std::vector<ServiceType> active_resources_;

        // Map given internally shared resource_ids to externally called resource id
        std::map<temoto_id::ID, temoto_id::ID> ids;


//		typedef std::pair<TemotoID::ID, std::shared_ptr<BaseResourceServer> ServerConnection;
//		std::vector<ServerConnection> internal_bindings_;

//		ResourceManager resource_manager_;

};

}

#endif
