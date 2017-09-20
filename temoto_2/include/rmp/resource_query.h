#ifndef RESOURCE_QUERY_H
#define RESOURCE_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>
#include "common/temoto_id.h"

namespace rmp
{

	// class for storing resource requests and hold their bingdings to clients
template <class ServiceMsgType>
class ResourceQuery{

	public:

		ResourceQuery()
		{

		}

		// special constructor for resource server
		ResourceQuery(const typename ServiceMsgType::Request& req)
		{
			addExternalClient(req.client_id, req.status_topic);
			msg_.request = req;
		}

		void addExternalClient(temoto_id::ID client_id, std::string status_topic)
		{
			external_clients_.emplace(client_id, status_topic);
		}

		// remove the external client and return how many is still connected
		size_t removeExternalClient(temoto_id::ID(client_id))
		{
			external_clients_.erase(client_id);
			return external_clients_.count;
		}


		void addinternalClient(std::string client_name, temoto_id::ID resource_id)
		{
			//try to insert to map
			auto ret = internal_clients_.emplace(client_name, std::set<temoto_id::ID>(resource_id));

			if (ret.second == false)
			{
				// client already exists, add current id to its set
				auto ins_ret = ret.first->second.insert(resource_id);
				if(ins_ret.second == false)
				{
					ROS_ERROR("An extreme badness has happened in ResourceQuery."
							"Somebody tried to bind same resource twice to a resource_server.");
				}

			}

		}


		const ServiceMsgType& getMsg()
		{
			return msg_;
		}

		void setMsgResponse(const typename ServiceMsgType::Response& res)
		{
			msg_.response = res;
		}

	private:

		// unique client name is mapped to set of resource ids
		std::map<std::string, std::set<temoto_id::ID>> internal_clients_;

		// unique client id is mapped with status topic
		std::map<temoto_id::ID, std::string> external_clients_;

		ServiceMsgType msg_;
	};
}


#endif
