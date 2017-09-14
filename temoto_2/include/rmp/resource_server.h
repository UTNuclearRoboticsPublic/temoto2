#ifndef RESOURCE_SERVER_H
#define RESOURCE_SERVER_H
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "rmp/base_resource_server.h"
#include "rmp/resource_manager.h"

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



		}

		~ResourceServer()
		{
		}

	//	void addResourceEntry(ResourceEntry<Service> entry)
	//	{
	//		entries_.push_back(entry);
	//	}


		bool wrappedLoadCallback(typename LoadService::Request& req, typename LoadService::Response& res)
		{
			// when a request with unassigned arrives, generate new id
			req.client_id = id_manager_.checkID(req.client_id);
			res.client_id = req.client_id;


			// call owner's registered callback
			bool ret = (owner_->*load_callback_)(req,res);

			// TODO: call compare function
			// if equal, then send back matched response with new id
			if (ret)
			{
				//entries_.emplace_back(req.id);
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
		TemotoID::IDManager id_manager_;


//		typedef std::pair<TemotoID::ID, std::shared_ptr<BaseResourceClient> ClientConnection;
//		std::vector<ClientConnection> connections_;

//		ResourceManager resource_manager_;

};

}

#endif
