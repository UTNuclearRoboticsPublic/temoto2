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

		ResourceClient(
				std::string ext_resource_manager_name,
				std::string ext_server_name,
				Owner* owner)
			: 
				ext_resource_manager_name_(ext_resource_manager_name),
				ext_server_name_(ext_server_name), 
				owner_(owner)
		{
			
			// Set client name according to the server, where it is connecting to.
			name_ = ext_resource_manager_name + "/" + ext_server_name;
            std::string unload_name = ext_resource_manager_name+"/unload";
            
			// Setup ROS service clients for loading and unloading the resource
			service_client_ = nh_.serviceClient<ServiceType>(name_);
			service_client_unload_ = nh_.serviceClient<temoto_2::UnloadResource>(unload_name);
			ROS_INFO("Created ResourceClient %s", name_.c_str());
			if(service_client_.exists())
			{
				ROS_INFO("Service client created and it exists!");
			}
		}

		~ResourceClient()
		{
			ROS_INFO("Destroyed ResourceClient %s", name_.c_str());
		}


		bool call(ServiceType& msg)
		{
			std::string prefix = "ResourceClient::call() [" + name_ + "]:";
			ROS_INFO("%s", prefix.c_str());

			// search for given request from active resources 
			auto msg_it = std::find_if(active_resources_.begin(), active_resources_.end(), 
					[&](const ServiceType& msg_el) -> bool {return msg_el.request == msg.request;}
                    ); 
			if(msg_it == active_resources_.end())
			{
				// New request
				ROS_INFO("%s New request, performing external call to %s", prefix.c_str(), service_client_.getService().c_str());
				if(service_client_.call(msg))
				{
					ROS_INFO("%s Service call returned true.", prefix.c_str()); 
					active_resources_.push_back(msg);
					msg_it  = std::prev(active_resources_.end()); // set iterator to the added element
				}
				else
				{
					ROS_ERROR("%s Service call returned false.", prefix.c_str()); 
					ROS_ERROR("%s Returning false.", prefix.c_str()); 
					return false; // something went wrong, return immediately
				}
			}
			else
			{
				// Equal request was found from stored list
				ROS_INFO("%s Existing request, using stored response", prefix.c_str());

				// Fill the response part with the one that was found from active_resources
				msg.response = msg_it->response;
			}

            // Generate id for the resource and add it to the map
            temoto_id::ID generated_id = id_manager_.generateID();
            ids.emplace(generated_id, msg_it->response.rmp.resource_id);

            // Update id in response part
            msg.response.rmp.resource_id = generated_id;

			ROS_ERROR("%s Returning true.", prefix.c_str()); 
			return true; 
		}

        
        void unloadResource(temoto_id::ID resource_id)
        {
			// search for given resource id to unload 
			auto found_msg = std::find_if(active_resources_.begin(), active_resources_.end(), 
					[&](const ServiceType& msg) -> bool {return msg.response.rmp.resource_id == resource_id;}
                    ); 
            if(found_msg != active_resources_.end())
            {
                sendUnload(resource_id)
                active_resources_.erase(found_msg);
            }
        }
        

        // Send unload service request to the server 
        void sendUnload(temoto_id::ID resource_id){
            temoto_2::UnloadResource unload_msg;
            unload_msg.request.server_name = ext_server_name_;
            unload_msg.request.resource_id = resource_id;
            if(!service_client_unload_.call(unload_msg))
            {
                ROS_ERROR("[ResourceClient::sendUnload] Call to %s failed.",
                        service_client_unload_.getService().c_str());
            }

        }


        size_t getConnectionCount() const {return active_resources_.size();}

        const std::string& getName() const {return name_;} 
        
        const std::string& getExtServerName() const {return ext_server_name_;} 


	private:

		std::string name_; /// The unique name of a resource client.
		std::string ext_server_name_; /// The name of the server client calls.
		std::string ext_resource_manager_name_; /// Name of resource manager where the server is located at.
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
