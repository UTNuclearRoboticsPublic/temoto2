#ifndef PROCESS_MANAGER_H
#define PROCESS_MANAGER_H

#include "core/common.h"
#include "process_manager/process_manager_errors.h"
#include "process_manager/process_manager_services.h"
#include "rmp/resource_manager.h"
#include <stdio.h> //pid_t TODO: check where pid_t actually is

namespace process_manager
{
	class ProcessManager
	{
		public:

			ProcessManager ();
			virtual ~ProcessManager ();

			std::string formatRequest(temoto_2::LoadProcess::Request& req);
			void formatResponse(temoto_2::LoadProcess::Response &res, int code, std::string message);
			bool loadCb(temoto_2::LoadProcess::Request &req, temoto_2::LoadProcess::Response &res);
			bool unloadCb(temoto_2::UnloadResource::Request &req, temoto_2::UnloadResource::Response &res);

			void update(const ros::TimerEvent& e);



		private:


			const std::string node_name_ = "process_manager";
			const std::string class_name_ = "ProcessManager";
			const std::vector<std::string> validActions = {"rosrun", "roslaunch"};

			std::map<pid_t, temoto_id::ID> running_processes_;

			ros::NodeHandle nh_;

			// Resource management protocol
			rmp::ResourceManager<ProcessManager> resource_manager_;

			//error::ErrorHandler error_handler_;

			// Listens for calls to start or kill processes
			//ros::ServiceServer spawn_kill_srv_;

			// Holds clients toi connect and send info to other (Sensor, Context, etc.) managers
			//ros::ServiceClient resource_status_client_;

	};
}

#endif
