#ifndef RESOURCE_SERVER_H
#define RESOURCE_SERVER_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_server.h"
#include "rmp/resource_query.h"


namespace rmp
{

template<class LoadService, class UnloadService, class Owner>
class ResourceServer : public BaseResourceServer<Owner>
{
	
	public:
		typedef bool(Owner::*LoadCbFuncType)(typename LoadService::Request&, typename LoadService::Response&);
		typedef bool(Owner::*UnloadCbFuncType)(typename UnloadService::Request&, typename UnloadService::Response&);

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
			load_server_ = nh_.advertiseService(this->name_, &ResourceServer<LoadService,UnloadService,Owner>::wrappedLoadCallback, this);
ROS_INFO("ResourceServer: created server for %s", this->name_.c_str());
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

		void registerInternalClient(temoto_id::ID resource_id)
		{
			if(!queries_.size())
			{
				ROS_ERROR("registerInternalClient called, but queries_ is empty");
				return;
			}
			queries_.back().addInternalClient(resource_id);
		};


		bool wrappedLoadCallback(typename LoadService::Request& req, typename LoadService::Response& res)
		{
			ROS_INFO("ResourceServer Load callback fired by client %ld, with return status_topic %s", req.client_id, req.status_topic.c_str());

			if(!owner_)
			{
				ROS_ERROR("ResourceServer Owner is NULL");
				return true;
			}

			// Register the client and get its ID from resource manager
			req.client_id = this->resource_manager_.registerExternalClient(req.client_id);
			res.client_id = req.client_id;


			// generate new id for the resource
			res.resource_id = res_id_manager_.generateID();

			ROS_INFO("ResourceServer Client id is %ld", req.client_id);


			//compare request messages
			if(isNewRequest(req))
			{
				ROS_INFO("new request");

				// equal message not found from queries_, add new query
				queries_.emplace_back(req);

				// set this server active in resource manager
				// when a client call is made from callback, the binding between active server
				// and the new loaded clients can be made automatically 
				this->activateServer();

				// call owner's registered callback
				bool ret = (owner_->*load_callback_)(req,res);

				ROS_INFO("ResourceServer resumed from callback");

				// update the query with the response message filled in the callback
				queries_.back().setMsgResponse(res);

				// restore active server to NULL in resource manager
				this->deactivateServer();

			}
			else
			{
				// found equal request, simply reqister this in the query
				// and respond with unique client_id.
				queries_.back().addExternalClient(req.client_id, req.status_topic);
			}

			return true; 
		}

        
        void unloadResource(temoto_2::UnloadResource::Request& req, temoto_2::UnloadResource::Response& res)
		{

		// check if this query binds any clients
	//		for (auto& query : queries_)
	//	   	{
	//			if(query.res.resource_id == req.resource_id)
	//			{


	//			}
	//		}
        //    
			// call owner's registered callback
            (owner_->*unload_callback_)(req,res);

			//queries_.removeExternalClient
		}

	private:

	//	std::vector<ResourceEntry<Service>> entries_;

		Owner* owner_;
		LoadCbFuncType load_callback_;	
		UnloadCbFuncType unload_callback_;	
		
		ros::ServiceServer load_server_;
		ros::NodeHandle nh_;
		temoto_id::IDManager res_id_manager_;

		std::vector<ResourceQuery<LoadService>> queries_;


//		typedef std::pair<TemotoID::ID, std::shared_ptr<BaseResourceClient> ClientConnection;
//		std::vector<ClientConnection> connections_;

//		ResourceManager resource_manager_;

};

}

#endif
