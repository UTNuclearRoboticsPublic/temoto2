#ifndef CONTEXT_MANAGER_CONTAINERS
#define CONTEXT_MANAGER_CONTAINERS

#include "temoto_2/ObjectContainer.h"

namespace context_manager
{

// Define the type of the data that is going to be synchronized
typedef std::vector<temoto_2::ObjectContainer> Objects;
typedef std::shared_ptr<temoto_2::ObjectContainer> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrs;

}

// ObjectContainer comparison operator
bool operator==(const temoto_2::ObjectContainer& ob1, const temoto_2::ObjectContainer& ob2);

#endif
