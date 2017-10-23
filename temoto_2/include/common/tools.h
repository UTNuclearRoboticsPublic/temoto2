#ifndef TOOLSX_H
#define TOOLSX_H

#include <string>

namespace common
{
/**
 * @brief Used for createing a prefix that indicates where the message is coming from
 * @param class_name
 * @param method_name
 * @return
 */
inline std::string generateLogPrefix(std::string subsys_name, std::string class_name,
                                     std::string method_name)
{
  std::string prefix = "[";
  if (subsys_name.size())
  {
    prefix += subsys_name + "/";
  }
  if (class_name.size())
  {
    prefix += class_name;
    if (method_name.size())
    {
      prefix += "::";
    }
  }
  prefix += method_name + "]";
  return prefix;
}
}

#endif
