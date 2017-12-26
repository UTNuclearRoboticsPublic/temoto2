#include "temoto_error/temoto_error.h"
#include "common/temoto_log_macros.h"

namespace error
{
ErrorHandler::ErrorHandler()
{
}

ErrorHandler::ErrorHandler(Subsystem subsystem, std::string log_group)
  : subsystem_(subsystem), log_group_(log_group)
{
}

ErrorStack ErrorHandler::create(Code code, const std::string& prefix, const std::string& message) const
{
  ErrorStack est;

  temoto_2::Error error;
  error.subsystem = static_cast<int>(subsystem_);
  error.code = static_cast<int>(code);
  error.prefix = prefix;
  error.message = message;
  error.stamp = ros::Time::now();

// Print the message out if the verbose mode is enabled
#ifdef __TEMOTO_ERROR_HANDLER_VERBOSE__
  TEMOTO_ERROR_STREAM(prefix << " " << message);
#endif

  est.push_back(error);
  return est;
}

ErrorStack ErrorHandler::forward(ErrorStack error_stack, const std::string& prefix) const
{
  temoto_2::Error error;
  error.subsystem = static_cast<int>(subsystem_);
  error.code = static_cast<int>(Code::FORWARDING);
  error.prefix = prefix;
  error.stamp = ros::Time::now();

// Print the message out if the verbose mode is enabled
#ifdef __TEMOTO_ERROR_HANDLER_VERBOSE__
  TEMOTO_ERROR_STREAM(prefix << " Forwarding.");
#endif

  error_stack.push_back(error);
  return error_stack;
}

void ErrorHandler::send(ErrorStack est) 
{
  // Create an ErrorStack message and publish it
  ros::Publisher error_publisher_ =
      n_.advertise<temoto_2::ErrorStack>("temoto_error_messages", 100);
  temoto_2::ErrorStack error_stack_msg;
  error_stack_msg.error_stack = est;
  error_publisher_.publish(error_stack_msg);
}

}  // error namespace


error::ErrorStack& operator+=(error::ErrorStack& es_lhs, const error::ErrorStack& es_rhs)
{
  es_lhs.insert(es_lhs.end(), es_rhs.begin(), es_rhs.end());
  return es_lhs;
}


std::ostream& operator<<(std::ostream& out, const temoto_2::Error& t)
{
  out << std::endl;
  out << "* code: " << t.code << std::endl;
  out << "* subsystem: " << t.subsystem << std::endl;
  out << RED << "* message: " << t.prefix << " " << t.message << RESET << std::endl;
  out << "* timestamp: " << t.stamp << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const error::ErrorStack& t)
{
  out << std::endl;
  // Start printing out the errors
  for (auto& err : t)
  {
    if (err.code != 0)
    {
      out << std::endl << " ------- Error Trace --------";
    }
    out << err;
  }
  return out;
}
