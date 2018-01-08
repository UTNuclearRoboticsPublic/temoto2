#ifndef TEMOTO_ERROR_H
#define TEMOTO_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"
#include "temoto_2/Error.h"
#include "temoto_2/ErrorStack.h"
#include "common/temoto_log_macros.h"
#include "common/console_colors.h"

namespace error
{

/**
 * @brief Enum that stores the subsystem codes
 */
enum class Subsystem : int
{
  AGENT,
  CONTEXT_MANAGER,
  HEALTH_MONITOR,
  SENSOR_MANAGER,
  ALGORITHM_MANAGER,
  ROBOT_MANAGER,
  OUTPUT_MANAGER,
  PROCESS_MANAGER,
  TASK
};

/**
 * @brief Enum that stores the error codes
 */
enum class Code : int
{
  FORWARDING,  // Indicate the forwarding type

  // Generic
  NULL_PTR,       // Pointer is null
  UNINITIALIZED,  // Object is not initialized

  // Config sync related
  YAML_ERROR,

  // Service related
  SERVICE_REQ_FAIL,     // Service request failed
  SERVICE_STATUS_FAIL,  // Service responded with FAILED status

  // Resource management
  RMP_FAIL,  // Something in RMP failed

  RESOURCE_LOAD_FAIL,    // Failed to load resource
  RESOURCE_UNLOAD_FAIL,  // Failed to unload resource
  RESOURCE_NOT_FOUND,    // Resource was not found

  // Core
  DESC_OPEN_FAIL,       // Failed to open the xml file
  DESC_NO_ROOT,         // Missing root element
  DESC_NO_ATTR,         // Attribute missing
  DESC_INVALID_ARG,     // Invalid/Corrupt arguments
  CLASS_LOADER_FAIL,    // Classloader failed to do its job
  FIND_TASK_FAIL,       // Failed to find tasks
  UNSPECIFIED_TASK,     // The task is unspecified
  NAMELESS_TASK_CLASS,  // The task is missing a class name
  NO_TASK_CLASS,        // Task handler could not find the task class

  // TTP
  BAD_ANY_CAST,       // Bad any cast
  NLP_INV_ARG,        // Invalid argument in Natural Language Processor
  NLP_BAD_INPUT,      // NLP was not able to make any sense from provided input text
  NLP_NO_TASK,        // Suitable task was not found
  NLP_DISABLED,       // NLP was tried to be used while it was disabled
  SUBJECT_NOT_FOUND,  // Subject was not found

  // Output manager
  RVIZ_OPEN_FAIL,           // Failed to open rviz
  PLUGIN_LOAD_FAIL,         // Failed to load rviz plugin
  PLUGIN_UNLOAD_FAIL,       // Failed to unload rviz plugin
  PLUGIN_GET_CONFIG_FAIL,   // Failed to get rviz plugin config
  PLUGIN_SET_CONFIG_FAIL,   // Failed to set rviz plugin config
  CONFIG_OPEN_FAIL,         // Failed to open the plugin config file
  ROBOT_FEATURE_NOT_FOUND,  // The requested feature is not available.
  ROBOT_VIZ_NOT_FOUND,      // Robot visualization problem.

  // Process manager
  PROCESS_SPAWN_FAIL,  // Failed to spawn new process
  PROCESS_KILL_FAIL,   // Failed to kill a process

  // Robot manager
  ROBOT_NOT_FOUND,   // The requested robot was not found from local and remote managers.
  ROBOT_NOT_LOADED,  // The requested robot is not loaded.

  // Robot manager
  SENSOR_NOT_FOUND,  // The requested robot was not found from local and remote managers.

  // Algorithm manager
  ALGORITHM_NOT_FOUND,  // The requested algorithm was not found from local and remote managers.
  NOT_INITIALIZED,      // TODO: same as UNINITIALIZED above?

  // Context Manager
  NO_TRACKERS_FOUND,

  UNHANDLED_EXCEPTION  // Unhandled exception
};

// Some random idea how to store error descriptions
static const std::map<Code, std::string> descriptions = {
  {Code::FORWARDING, "Forwarding" },
  {Code::PROCESS_KILL_FAIL, "Node kill failed heavily, it was hit by extreme badness" }
};


/**
 * @brief ErrorStack
 */
typedef std::vector<temoto_2::Error> ErrorStack;

#define __TEMOTO_ERROR_HANDLER_VERBOSE__ TRUE

#define CREATE_ERROR(code, ...) this->error_handler_.create(code, TEMOTO_LOG_PREFIX, error::ErrorHandler::formatToString(__VA_ARGS__))

#define FORWARD_ERROR(error_stack) this->error_handler_.forward(error_stack, TEMOTO_LOG_PREFIX)

#define SEND_ERROR(error_stack) this->error_handler_.send(error_stack)


/**
 * @brief The ErrorHandler class
 */
class ErrorHandler
{
public:
  ErrorHandler();

  ErrorHandler(Subsystem subsystem, std::string log_group);

  /**
   * @brief Creates the ErrorStack object.
   * @param code Code for classifying the error.
   * @param prefix Prefix describing where the error was created.
   * @param message A brief description of what went wrong.
   */
  ErrorStack create(Code code, const std::string& prefix, const std::string& message) const; 


  /**
   * @brief Appends the existing error stack with a prefix.
   * @param error_stack Error stack to which the prefix is appended.
   * @param prefix Prefix describing where the error is forwarded.
   */
  ErrorStack forward(ErrorStack error_stack, const std::string& prefix) const;

  /**
   * @brief Publishes the error_stack
   * @param error_stack
   */
  void send(ErrorStack error_stack);


  static std::string formatToString(const char* fmt, ...)
  {
    boost::shared_array<char> buffer;
    size_t size = 0;
    va_list args;
    va_start(args, fmt);
    ros::console::vformatToBuffer(buffer, size, fmt, args);
    va_end(args);
    return std::string(buffer.get(), size);
  }

  static std::string formatToString(const std::string& s) 
  {
    return s;
  }


private:
  Subsystem subsystem_;

  std::string log_group_;

  ros::NodeHandle n_;
};

}  // end of error namespace


/**
 * @brief Define + operator to append some other ErrorStack to this stack.
 */
error::ErrorStack& operator+=(error::ErrorStack& er_lhs, const error::ErrorStack& es_rhs);

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const temoto_2::Error& t);

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const error::ErrorStack& t);

#endif
