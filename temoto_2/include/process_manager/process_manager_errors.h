#ifndef PROCESS_MANAGER_ERRORS_H
#define PROCESS_MANAGER_ERRORS_H

#include "temoto_error/temoto_error.h"

namespace process_manager
{
    enum class error_type : int
    {
        FORWARDING = 0,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        PROCESS_SPAWN_FAIL,             // Failed to spawn new process
        PROCESS_KILL_FAIL              // Failed to kill a process
    };

	static std::map<error_type, std::string> error_descs = {
		{error_type::FORWARDING, "Forwarding"},
		{error_type::PROCESS_KILL_FAIL, "Node kill failed heavily, it was hit by extreme badness"}
	};
		
		
}

#endif
