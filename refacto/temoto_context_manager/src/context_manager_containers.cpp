#include "temoto_context_manager/context_manager_containers.h"

namespace temoto_context_manager
{

// ObjectContainer comparison operator
bool operator==(const ObjectContainer& ob1, const ObjectContainer& ob2)
{
  if(ob1.name != ob2.name)
  {
    return false;
  }

  return true;
}
} // temoto_context_manager namespace
