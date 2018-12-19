#ifndef TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_H
#define TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_H

#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include <stdio.h> //pid_t TODO: check where pid_t actually is
#include <mutex>
#include <sys/stat.h>

namespace temoto_er_manager
{
	class ERManager : public temoto_core::BaseSubsystem
	{
		public:

			ERManager ();
			virtual ~ERManager ();

			std::string formatRequest(temoto_er_manager::LoadExtResource::Request& req);
			void formatResponse(temoto_er_manager::LoadExtResource::Response &res, int code, std::string message);
			void loadCb(temoto_er_manager::LoadExtResource::Request &req, temoto_er_manager::LoadExtResource::Response &res);
			void unloadCb(temoto_er_manager::LoadExtResource::Request &req, temoto_er_manager::LoadExtResource::Response &res);

			void update(const ros::TimerEvent& e);

      const std::string& getName() const
      {
        return log_subsys_;
      }

    private:


      std::string log_class_, log_subsys_, log_group_;

      // TODO: This section should be replaced by a single container which also holds
      // the state of each process.
      std::vector<temoto_er_manager::LoadExtResource> loading_processes_;
      std::map<pid_t, temoto_er_manager::LoadExtResource> running_processes_;
      std::map<pid_t, temoto_er_manager::LoadExtResource> failed_processes_;
      std::vector<pid_t> unloading_processes_;

      std::mutex loading_mutex_;
      std::mutex unloading_mutex_;
      std::mutex running_mutex_;

			ros::NodeHandle nh_;

			// Resource management protocol
			temoto_core::rmp::ResourceManager<ERManager> resource_manager_;

			// Listens for calls to start or kill processes
			//ros::ServiceServer spawn_kill_srv_;

			// Holds clients toi connect and send info to other (Sensor, Context, etc.) managers
			//ros::ServiceClient resource_status_client_;
      void waitForLock(std::mutex& m);
      inline bool executableExists (const std::string& name) {
          struct stat buffer;
          return (stat(name.c_str(), &buffer) == 0);
      }
  };
}

#endif
