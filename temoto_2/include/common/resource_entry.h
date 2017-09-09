#ifndef RESOURCE_ENTRY_H
#define RESOURCE_ENTRY_H

#include "common/temoto_id.h"

namespace rap
{

	template<class Service>
	class ResourceEntry
	{
		private:
			TemotoID::ID id_;
			Service service_;
	};

}

#endif
