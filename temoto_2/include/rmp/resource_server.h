#ifndef RESOURCE_SERVER_H
#define RESOURCE_SERVER_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_server.h"
#include "rmp/resource_query.h"


namespace rmp
{

template<class ServiceType, class Owner>
class ResourceServer : public BaseResourceServer<Owner>
{
	
	public:
		typedef void(Owner::*LoadCbFuncType)(typename ServiceType::Request&, typename ServiceType::Response&);
		typedef void(Owner::*UnloadCbFuncType)(typename ServiceType::Request&, typename ServiceType::Response&);

		ResourceServer(std::string name,
				LoadCbFuncType load_cb,
				UnloadCbFuncType unload_cb,
				Owner* owner,
				ResourceManager<Owner>& resource_manager
				) :
				BaseResourceServer<Owner>(name, resource_manager),
				load_callback_(load_cb), 
				unload_callback_(unload_cb),
				owner_(owner)
		{
            std::string rm_name = this->resource_manager_.getName();
			load_server_ = nh_.advertiseService(rm_name + "/" + this->name_, &ResourceServer<ServiceType,Owner>::wrappedLoadCallback, this);
ROS_INFO("ResourceServer constructed, listening on %s", this->load_server_.getService().c_str());
		}

		~ResourceServer()
		{
			ROS_INFO("ResourceServer[%s] destroyed.", this->name_.c_str());
		}


		void registerInternalClient(temoto_id::ID resource_id)
		{
			if(!queries_.size())
			{
				ROS_ERROR("registerInternalClient called, but queries_ is empty");
				return;
			}
			queries_.back().addInternalClient(resource_id);
		};


		bool wrappedLoadCallback(typename ServiceType::Request& req, typename ServiceType::Response& res)
		{
			std::string prefix = "ResourceServer::wrappedLoadCallback() [" + this->name_ + "]:";
			ROS_INFO("%s Got query with status_topic: '%s'", prefix.c_str(), req.rmp.status_topic.c_str());

			if(!owner_)
			{
				ROS_ERROR("%s ResourceServer Owner is NULL", prefix.c_str());
				return true;
			}

			// generate new id for the resource
			res.rmp.resource_id = res_id_manager_.generateID();

			ROS_INFO("%s Created resource with id %ld", prefix.c_str(), res.rmp.resource_id);


			// New or existing query? Check it out with this hi-tec lambda function :)
			auto found_query = std::find_if(queries_.begin(), queries_.end(), 
					[&req](const ResourceQuery<ServiceType>& query) -> bool { return query.getMsg().request == req;  }); 

			if(found_query == queries_.end())
			{
				ROS_INFO("%s New query, going for owners callback", prefix.c_str());

				// equal message not found from queries_, add new query
				queries_.emplace_back(req, res);

				// set this server active in resource manager
				// when a client call is made from callback, the binding between active server
				// and the new loaded clients can be made automatically 
				this->activateServer();

				// call owner's registered callback
				(owner_->*load_callback_)(req,res);

				ROS_INFO("%s Resumed from owners callback", prefix.c_str());

				// update the query with the response message filled in the callback
				queries_.back().setMsgResponse(res);

				// restore active server to NULL in resource manager
				this->deactivateServer();
			}
			else
			{
				// found equal request, simply reqister this in the query
				// and respond with unique resoure_id.
				ROS_INFO("%s Existing query, linking to the found query.", prefix.c_str());
				queries_.back().addExternalClient(req,res);
			}

			ROS_INFO("%s Returning true.", prefix.c_str());
			return true; 
		}

        
        void unloadResource(temoto_2::UnloadResource::Request& req, temoto_2::UnloadResource::Response& res)
		{
			// check if this query binds any clients
			const temoto_id::ID resource_id = req.resource_id;
			const auto found_query_it = std::find_if(queries_.begin(), queries_.end(), 
					[resource_id](const ResourceQuery<ServiceType>& query) -> bool {return query.externalClientExists(resource_id);}
					); 
			if (found_query_it != queries_.end())
			{
				// Query found, try to remove resource from it.
				size_t clients_remaining = found_query_it->removeExternalClient(req.resource_id);
				if (clients_remaining <= 0)
				{
					// last resource removed, execute owner's unload callback and remove the query from our list
					typename ServiceType::Request orig_req = found_query_it->getMsg().request;	
					typename ServiceType::Response orig_res = found_query_it->getMsg().response;	
					(owner_->*unload_callback_)(orig_req, orig_res);

					/// TODO: Do or do not something with the response part?
                    
                    // Send unload command to all connected clients...
                    for (auto& map_el : found_query_it->getInternalClients())
                    {
                        for (auto& set_el : map_el.second)
                            this->resource_manager_.unloadClient(map_el.first, set_el);
                    }
                    
                    // Finally, remove the found query.
					queries_.erase(found_query_it);
				}
			}
		};

	private:

		Owner* owner_;
		LoadCbFuncType load_callback_;	
		UnloadCbFuncType unload_callback_;	
		
		ros::ServiceServer load_server_;
		ros::NodeHandle nh_;
		temoto_id::IDManager res_id_manager_;

		std::vector<ResourceQuery<ServiceType>> queries_;

};

}

#endif
