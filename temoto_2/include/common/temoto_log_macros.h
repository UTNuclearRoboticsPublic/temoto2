#ifndef TEMOTO_LOG_MACROS_H
#define TEMOTO_LOG_MACROS_H

#include "common/tools.h"
#include <string>

#define TEMOTO_CONSOLE_NAME ROSCONSOLE_ROOT_LOGGER_NAME "."+::common::getTemotoNamespace()+"."+this->log_group_
#define TEMOTO_DEBUG(...) TEMOTO_LOG(::ros::console::levels::Debug, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_INFO(...) TEMOTO_LOG(::ros::console::levels::Info, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_WARN(...) TEMOTO_LOG(::ros::console::levels::Warn, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_ERROR(...) TEMOTO_LOG(::ros::console::levels::Error, TEMOTO_CONSOLE_NAME, __VA_ARGS__)

#define TEMOTO_DEBUG_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Debug, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_INFO_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Info, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_WARN_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Warn, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_ERROR_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Error, TEMOTO_CONSOLE_NAME, __VA_ARGS__)

#define TEMOTO_LOG_PREFIX ("::"+::common::getTemotoNamespace()+"/"+this->subsystem_name_+"/"+this->class_name_+"::"+__func__).c_str()

#define TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(filter, ...) \
::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __FILE__, __LINE__, TEMOTO_LOG_PREFIX, __VA_ARGS__)

#define TEMOTO_PRINT_AT_LOCATION(...) \
TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(0, __VA_ARGS__)

#define TEMOTO_LOG_FILTER(filter, level, name, ...) \
do \
{ \
  ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
  if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (filter)->isEnabled()) \
  { \
    TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(filter, __VA_ARGS__); \
  } \
} while(0)
 
#define TEMOTO_LOG_COND(cond, level, name, ...) \
  do \
  { \
   ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
   \
   if (ROS_UNLIKELY(__rosconsole_define_location__enabled)) \
   { \
    TEMOTO_PRINT_AT_LOCATION(__VA_ARGS__); \
   } \
  } while(0)

#define TEMOTO_LOG(level, name, ...) TEMOTO_LOG_COND(true, level, name, __VA_ARGS__)








#endif
