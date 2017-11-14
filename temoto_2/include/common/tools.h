#ifndef TOOLSX_H
#define TOOLSX_H

#include <ros/ros.h>
#include <string>
#include <stdlib.h> // for getenv


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

inline const std::string getTemotoNamespace()
{
//  std::string temoto_name_str;
//
//  char* temoto_name_char = NULL;
//  temoto_name_char = getenv("TEMOTO_NAMESPACE");
//  if (temoto_name_char != NULL)
//  {
//    temoto_name_str = temoto_name_char;
//  }
//  else
//  {
//    temoto_name_str = "temoto_2_default";
//  }

  std::string temoto_ns = ::ros::this_node::getNamespace();
  temoto_ns = ::ros::names::clean(temoto_ns); // clean up double and trailing slashes
  if(temoto_ns.size()>0 and temoto_ns[0] == '/')
  {
    temoto_ns.erase(temoto_ns.begin()); // remove the leading '/'
  }

  return temoto_ns;
}
}

#endif
