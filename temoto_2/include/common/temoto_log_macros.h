#ifndef TEMOTO_LOG_MACROS_H
#define TEMOTO_LOG_MACROS_H

#define TEMOTO_CONSOLE_PREFIX ROSCONSOLE_ROOT_LOGGER_NAME "."+::common::getTemotoName()+"."+this->log_group_
#define TEMOTO_DEBUG(...) ROS_LOG(::ros::console::levels::Debug, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_INFO(...) ROS_LOG(::ros::console::levels::Info, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_WARN(...) ROS_LOG(::ros::console::levels::Warn, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_ERROR(...) ROS_LOG(::ros::console::levels::Error, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)

#define TEMOTO_DEBUG_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Debug, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_INFO_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Info, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_WARN_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Warn, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)
#define TEMOTO_ERROR_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Error, TEMOTO_CONSOLE_PREFIX, __VA_ARGS__)


#endif
