#ifndef CLIENT_QUERY_H
#define CLIENT_QUERY_H

#include <string>
#include <vector>
#include <map>
#include <utility>
#include "common/temoto_id.h"

namespace rmp
{


	// class for storing resource requests and hold their bingdings to external servers 
template <class ServiceMsgType>
class ClientQuery{

	public:

		ClientQuery()
		{

		}


		// special constructor for resource client
		ClientQuery(const ServiceMsgType& msg) : msg_(msg) {}
		

        void addInternalResource(temoto_id::ID resource_id, std::string& server_name)
        {
            //try to insert to map
            auto ret = internal_resources_.emplace(resource_id, server_name);

            if (!ret.second)
            {
                ROS_ERROR("[ClientQuery::addInternalResource]: not allowed to add internal resources with identical ids.");
            }
        }


		// remove the internal resource from this query and return how many are still connected
		size_t removeInternalResource(temoto_id::ID resource_id)
		{
			auto it = internal_resources_.find(resource_id);
			if(it == internal_resources_.end())
			{
				throw "[ClientQuery::removeInternalResource] Resource_id not found!";
			}
			std::string caller_name = it->second;

			/// Erase resource_id from internal resource map.
			internal_resources_.erase(it);

			/// Count how many internal connections we have left with the erased caller.
			size_t cnt = count_if(internal_resources_.begin(), internal_resources_.end(),
					[&](const std::pair<temoto_id::ID, std::string>& r) -> bool 
					{return r.second == caller_name;}
					);

			return cnt;
		}


		// Check if given internal resource_id is attached to this query.
		bool internalResourceExists(temoto_id::ID resource_id) const
		{
			return internal_resources_.find(resource_id) != internal_resources_.end(); 
		}


		const ServiceMsgType& getMsg() const
		{
			return msg_;
		}

		
		const temoto_id::ID getExternalId() const
		{
			return msg_.response.rmp.resource_id;
		}

        const std::map<temoto_id::ID, std::string> getInternalResources() const {return internal_resources_;}


	private:

		// internal resource ids and their callers name
        std::map<temoto_id::ID, std::string> internal_resources_;

		ServiceMsgType msg_; /// Store request and response, note that RMP specific fields (resource_id, topic, ...) are related to first query and are not intended to be used herein.
	};
}


#endif
