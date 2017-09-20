#ifndef RESOURCE_SERVER_H
#define RESOURCE_SERVER_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_server.h"
#include "rmp/resource_manager.h"
#include "rmp/resource_query.h"


namespace rmp
{

template <class Owner>
class ResourceManager;

template<class LoadService, class UnloadService, class Owner>
class ResourceServer : public BaseResourceServer
{
	
	public:
		typedef bool(Owner::*LoadCbFuncType)(typename LoadService::Request&, typename LoadService::Response&);
		typedef bool(Owner::*UnloadCbFuncType)(typename UnloadService::Request&, typename UnloadService::Response&);

		ResourceServer(std::string name,
				LoadCbFuncType load_cb,
				UnloadCbFuncType unload_cb,
				Owner* owner,
				ResourceManager<Owner>& resource_manager
				) : name_(name), load_callback_(load_cb), unload_callback_(unload_cb), 
					owner_(owner), resource_manager_(resource_manager)
		{
			load_server_ = nh_.advertiseService(name_+"/load", &ResourceServer<LoadService,UnloadService,Owner>::wrappedLoadCallback, this);
			unload_server_ = nh_.advertiseService(name_+"/unload", &ResourceServer<LoadService,UnloadService,Owner>::wrappedUnloadCallback, this);

ROS_INFO("ResourceServer: created load and unload server for %s", name_.c_str());

		}

		~ResourceServer()
		{
		}

	//	void addResourceEntry(ResourceEntry<Service> entry)
	//	{
	//		entries_.push_back(entry);
	//	}


		bool isNewRequest(typename LoadService::Request& req)
		{

			for(auto q : queries_)
			{
				if(q.getMsg().request == req)
				{
					return false;
				}
			}

			return true;
		};


		bool wrappedLoadCallback(typename LoadService::Request& req, typename LoadService::Response& res)
		{
			ROS_INFO("ResourceServer Load callback fired by client %ld, with return status_topic %s", req.client_id, req.status_topic.c_str());
			
			// Register the client and get its ID from resource manager
			req.client_id = resource_manager_.registerInternalClient(req.client_id);
			res.client_id = req.client_id;


			// generate new id for the resource
			res.resource_id = res_id_manager_.generateID();

			ROS_INFO("ResourceServer Client id is %ld", req.client_id);


			//compare request messages
			if(isNewRequest(req))
			{
				queries_.back().addExternalClient(req.client_id, req.status_topic);
			}
			else
			{
				// equal message not found from queries_, add new query
				queries_.emplace_back(req);
				
				// call owner's registered callback
				bool ret = (owner_->*load_callback_)(req,res);

			ROS_INFO("ResourceServer resumed from callback");


				// update the query with users response
				queries_.back().setMsgResponse(res);
				
			}

			return true; 
		}

		bool wrappedUnloadCallback(typename UnloadService::Request& req, typename UnloadService::Response& res)
		{
			// check if client_id has any resources loaded
		//	for (auto& entry : entries_)
		//   	{
		//		if(entry.req.client_id == req.client_id)
		//		{

		//		}
		//	}
            
			// call owner's registered callback
			bool ret = (owner_->*unload_callback_)(req,res);

			//queries_.removeExternalClient


			return true; 
		}

	private:

	//	std::vector<ResourceEntry<Service>> entries_;

		std::string name_;
		Owner* owner_;
		ResourceManager<Owner>& resource_manager_;
		LoadCbFuncType load_callback_;	
		UnloadCbFuncType unload_callback_;	
		
		ros::ServiceServer load_server_;
		ros::ServiceServer unload_server_;
		ros::NodeHandle nh_;
		temoto_id::IDManager res_id_manager_;

		std::vector<ResourceQuery<LoadService>> queries_;


//		typedef std::pair<TemotoID::ID, std::shared_ptr<BaseResourceClient> ClientConnection;
//		std::vector<ClientConnection> connections_;

//		ResourceManager resource_manager_;

};

}

#endif
