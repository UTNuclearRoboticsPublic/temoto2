#ifndef CONTEXT_MANAGER_CONTAINERS_H
#define CONTEXT_MANAGER_CONTAINERS_H

#include "temoto_2/ObjectContainer.h"
#include "temoto_core/common/topic_container.h"

namespace context_manager
{

// Define the type of the data that is going to be synchronized
typedef std::vector<temoto_2::ObjectContainer> Objects;
typedef std::shared_ptr<temoto_2::ObjectContainer> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrs;

// ObjectContainer comparison operator
bool operator==(const temoto_2::ObjectContainer& ob1, const temoto_2::ObjectContainer& ob2);

}

#endif
