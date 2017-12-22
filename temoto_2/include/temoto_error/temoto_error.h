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
const int FORWARDING_CODE = 0;

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
  TASK
};

/**
 * @brief ErrorStack
 */
typedef std::vector<temoto_2::Error> ErrorStack;

#define __TEMOTO_ERROR_HANDLER_VERBOSE__ TRUE

#define CREATE_ERROR(code, message) error_handler_.create(code, TEMOTO_LOG_PREFIX, message)

#define FORWARD_ERROR(error_stack) error_handler_.forward(error_stack, TEMOTO_LOG_PREFIX)

#define ERROR_SEND()

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
  ErrorStack create(int code, const std::string& prefix, const std::string& message);

  /**
   * @brief Appends the existing error stack with a prefix.
   * @param error_stack Error stack to which the prefix is appended.
   * @param prefix Prefix describing where the error is forwarded.
   */
  ErrorStack forward(ErrorStack error_stack, const std::string& prefix);

  /**
   * @brief Publishes the error_stack
   * @param error_stack
   */
  void send(ErrorStack error_stack);

private:
  Subsystem subsystem_;

  std::string log_group_;

  ros::NodeHandle n_;

  ros::Publisher error_publisher_;
};

}  // end of error namespace

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
