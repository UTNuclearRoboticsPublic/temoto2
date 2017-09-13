#ifndef RESOURCE_H
#define RESOURCE_H
#include "ros/ros.h"
#include "common/base_resource.h"
#include "common/temoto_id.h"

namespace rmp
{

template<class Service, class Owner>
class Resource : public BaseResource
{
	
	public:
		typedef bool(Owner::*CbFuncType)(typename Service::Request&, typename Service::Response&);

		Resource(std::string service_name,
				CbFuncType callback,
				Owner* owner)
		{
			server_ = nh_.advertiseService(service_name, &Resource<Service,Owner>::wrappedCallback, this);
			owner_ = owner;
			callback_ = callback;
		}

		~Resource()
		{
		}

		void addResourceEntry(ResourceEntry<Service> entry)
		{
			entries_.push_back(entry);
		}

		bool wrappedCallback(typename Service::Request& req, typename Service::Response& res)
		{
			// when a request with unassigned arrives, generate new id
			req.id = id_manager_.checkID(req.id);
			res.id = req.id;


			// call owner's registered callback
			bool ret = (owner_->*callback_)(req,res)
				
// TODO: call compare function
// if equal, then send back matched response with new id
			if (ret)
			{

				entries_.emplace_back(req.id);
			}

			return ret; 
		}

	private:

		std::vector<ResourceEntry<Service>> entries_;

		ros::ServiceServer server_;
		ros::NodeHandle nh_;
		TemotoID::IDManager id_manager_;

		CbFuncType star_callback_;	
		CbFuncType stop_callback_;	
		Owner* owner_;

};

}

#endif
