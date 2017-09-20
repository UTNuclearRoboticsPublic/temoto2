#ifndef NODE_MANAGER_H
#define NODE_MANAGER_H

#include "core/common.h"
#include "node_manager/node_manager_errors.h"
#include "node_manager/node_manager_services.h"
#include "rmp/resource_manager.h"

namespace node_manager
{
	class NodeManager
	{
		public:

			NodeManager ();
			virtual ~NodeManager ();

			std::string formatRequest(temoto_2::LoadResource::Request& req);
			bool compareRequest(temoto_2::LoadResource::Request& req1,
					temoto_2::LoadResource::Request& req2,
					std::string action);

			void formatResponse(temoto_2::LoadResource::Response &res, int code, std::string message);
			bool loadCb(temoto_2::LoadResource::Request &req, temoto_2::LoadResource::Response &res);
			bool unloadCb(temoto_2::UnloadResource::Request &req, temoto_2::UnloadResource::Response &res);

			void update(const ros::TimerEvent& e);



		private:


			const std::string node_name_ = "node_manager";
			const std::string class_name_ = "NodeManager";
			const std::vector<std::string> validActions = {"rosrun", "roslaunch", "kill"};

			std::map<pid_t, temoto_2::LoadResource::Request> running_processes_;

			ros::NodeHandle nh_;

			// Resource management protocol
			rmp::ResourceManager<NodeManager> resource_manager_;

			//error::ErrorHandler error_handler_;

			// Listens for calls to start or kill processes
			//ros::ServiceServer spawn_kill_srv_;

			// Holds clients toi connect and send info to other (Sensor, Context, etc.) managers
			//ros::ServiceClient resource_status_client_;

	};
}

#endif
