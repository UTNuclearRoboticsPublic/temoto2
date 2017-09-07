#ifndef NODE_MANAGER_ERRORS_H
#define NODE_MANAGER_ERRORS_H

#include "base_error/base_error.h"

namespace node_manager
{
    enum class error_type : int
    {
        FORWARDING = 0,     // Code 0 errors always indicate the forwarding type, hence it has to be set manually to zero

        NODE_SPAWN_FAIL,             // Failed to spawn new node
        NODE_KILL_FAIL              // Failed to kill a node
    };
static int fakk;
//	std::map<error_type, std::string> error_descs;
//   	= {
//		{error_type::FORWARDING, "Forwarding"},
//		{error_type::NODE_KILL_FAIL, "Node kill failed heavily, it was hit by extreme badness"}
//	};
		

		
}

#endif
