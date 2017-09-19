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
		void addExternalClient(std::string status_topic, temoto_id::ID resource_id)
		{
			external_clients_.emplace(resource_id, status_topic);
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


		ServiceMsgType getMsg()
		{
			return msg;
		}

	private:
		std::map<std::string, std::set<temoto_id::ID>> internal_clients_;
		std::map<temoto_id::ID, std::string> external_clients_;
			ServiceMsgType msg;
	};
}


#endif
