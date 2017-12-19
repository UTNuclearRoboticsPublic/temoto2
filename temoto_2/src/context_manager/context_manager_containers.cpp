#include "context_manager/context_manager_containers.h"

// ObjectContainer comparison operator
bool operator==(const temoto_2::ObjectContainer& ob1, const temoto_2::ObjectContainer& ob2)
{
  if(ob1.name != ob2.name)
  {
    return false;
  }

  return true;
}
