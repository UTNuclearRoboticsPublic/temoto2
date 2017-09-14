#ifndef RESOURCE_ENTRY_H
#define RESOURCE_ENTRY_H

#include "common/temoto_id.h"

namespace rmp
{

	template<class Service>
	class ResourceEntry
	{
		public:
			ResourceEntry(temoto_id::ID id, Service service):id_(id), service_(service){}

		private:
			temoto_id::ID id_;
			Service service_;
	};

}

#endif
