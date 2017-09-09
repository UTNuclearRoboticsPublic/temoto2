#ifndef RESOURCE_H
#define RESOURCE_H
#include "ros/ros.h"
#include "common/base_resource.h"

namespace rap
{

template<class Service, class Owner>
class Resource : public BaseResource
{
	
	public:

		Resource(std::string service_name,
				bool(Owner::*callback)(typename Service::Request&, typename Service::Response&),
				Owner* owner)
		{
			server_ = nh_.advertiseService(service_name, callback, owner);

		}

		~Resource()
		{
		}

		void addResourceEntry(ResourceEntry<Service> entry)
		{
			entries_.push_back(entry);
		}

	private:

		std::vector<ResourceEntry<Service>> entries_;

		ros::ServiceServer server_;
		ros::NodeHandle nh_;

};

}

#endif
