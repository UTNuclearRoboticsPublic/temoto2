#include "temoto_error/temoto_error.h"
#include "common/temoto_log_macros.h"

namespace error
{

/* * * * * * * * * * * * * * * * * * *
 *
 *    ErrorStackUtil implementations
 *
 * * * * * * * * * * * * * * * * * * */

/*
 * Constructor
 */
ErrorStackUtil::ErrorStackUtil(int code,
                               Subsystem subsystem,
                               Urgency urgency,
                               std::string message,
                               ros::Time timeStamp)
{
    // Print the message to the console
    ROS_ERROR("%s", message.c_str());

    push( code,
          subsystem,
          urgency,
          message,
          timeStamp);
}

/*
 * Constructor
 */
ErrorStackUtil::ErrorStackUtil(int code,
                               Subsystem subsystem,
                               Urgency urgency,
                               std::string message)
{
    // Print the message to the console
    ROS_ERROR("%s", message.c_str());

    push( code,
          subsystem,
          urgency,
          message,
          ros::Time::now());
}

/*
 * Get stack
 */
ErrorStack ErrorStackUtil::getStack()
{
    return this->error_stack_;
}

/*
 * Push
 */
void ErrorStackUtil::push(int code,
                          Subsystem subsystem,
                          Urgency urgency,
                          std::string message,
                          ros::Time timeStamp )
{
    this->error_stack_.push_back( initBaseError( code,
                                                 subsystem,
                                                 urgency,
                                                 message,
                                                 timeStamp));
}

/*
 * Push
 */
void ErrorStackUtil::push ( temoto_2::Error base_error )
{
    this->error_stack_.push_back ( base_error );
}

/*
 * Initialize Base Error
 */
temoto_2::Error ErrorStackUtil::initBaseError(int code,
                                              Subsystem subsystem,
                                              Urgency urgency,
                                              std::string message,
                                              ros::Time stamp)
{
    temoto_2::Error error_msg;
    error_msg.code = static_cast<int>(code);
    error_msg.subsystem = static_cast<int>(subsystem);
    error_msg.urgency = static_cast<int>(urgency);
    error_msg.message = message;
    error_msg.stamp = stamp;

    return error_msg;
}

/*
 * Forward error
 */
void ErrorStackUtil::forward ( std::string from_where )
{
    temoto_2::Error fwd_err = this->error_stack_.back();
    fwd_err.code = 0;
    fwd_err.message = from_where + " FORWARDING";

    this->push( fwd_err );
}


/* * * * * * * * * * * * * * * *
 *
 *    ErrorHandler implementations
 *
 * * * * * * * * * * * * * * * */

ErrorHandler::ErrorHandler()
{
  error_publisher_ = n_.advertise<temoto_2::ErrorStack>("temoto_error_messages", 100);
}

ErrorHandler::ErrorHandler(Subsystem subsystem, std::string log_group)
 : subsystem_(subsystem)
 , log_group_(log_group)
{
  error_publisher_ = n_.advertise<temoto_2::ErrorStack>("temoto_error_messages", 100);
}

void ErrorHandler::createAndThrow(int code, std::string prefix, std::string message)
{
  ErrorStack est;
  temoto_2::Error error;

  error.subsystem = static_cast<int>(subsystem_);
  error.code = code;
  error.prefix = prefix;
  error.message = message;
  error.stamp = ros::Time::now();

  // Print the message out if the verbose mode is enabled
  if (VERBOSE)
  {
    TEMOTO_ERROR_STREAM(prefix << " " << message);
  }

  est.push_back(error);
  throw est;
}

void ErrorHandler::forwardAndThrow(ErrorStack& est, std::string prefix)
{
  temoto_2::Error error;

  error.subsystem = static_cast<int>(subsystem_);
  error.code = FORWARDING_CODE;
  error.prefix = prefix;
  error.stamp = ros::Time::now();

  // Print the message out if the verbose mode is enabled
  if (VERBOSE)
  {
    TEMOTO_ERROR_STREAM(prefix << " " << "Forwarding.");
  }

  est.push_back(error);
  throw est;
}

bool ErrorHandler::gotUnreadErrors() const
{
    return this->newErrors_;
}

void ErrorHandler::append( ErrorStack est )
{
    // Set the "newErrors" flag and append
    this->newErrors_ = true;
    this->errorStack_.insert(std::end(this->errorStack_), std::begin(est), std::end(est));

    // Create an ErrorStack message and publish it
    temoto_2::ErrorStack error_stack_msg;
    error_stack_msg.errorStack = est;

    error_publisher_.publish( error_stack_msg );
}

void ErrorHandler::append( ErrorStackUtil err_stck_util )
{
    append( err_stck_util.getStack() );
}

ErrorStack ErrorHandler::read()
{
    // Set the "newErrors" flag and return the stack
    this->newErrors_ = false;
    return this->errorStack_;
}

ErrorStack ErrorHandler::readSilent() const
{
    return this->errorStack_;
}

ErrorStack ErrorHandler::readAndClear()
{
    // Set the "newErrors" flag
    this->newErrors_ = false;

    // Create a copy of the errorStack_
    ErrorStack errorStackCopy( this->errorStack_ );

    // Clear the errorStack_
    this->errorStack_.clear();

    // Return the copy
    return errorStackCopy;
}
} // error namespace

std::ostream& operator<<(std::ostream& out, const temoto_2::Error& t)
{
    out << std::endl;
    out << "* code: " << t.code << std::endl;
    out << "* subsystem: " << t.subsystem << std::endl;
    out << "* urgency: " ;

    error::Urgency urg = static_cast<error::Urgency>(t.urgency);
    if ( urg == error::Urgency::LOW )
        out << "LOW" << std::endl;

    else if ( urg == error::Urgency::MEDIUM )
        out << "MEDIUM" << std::endl;

    else if ( urg == error::Urgency::HIGH )
        out << "HIGH" << std::endl;

    out << RED << "* message: " << t.prefix << " " << t.message << RESET << std::endl;
    out << "* timestamp: " << t.stamp  << std::endl;

    return out;
}

std::ostream& operator<<(std::ostream& out, const error::ErrorStack& t)
{
    out << std::endl;

    // Start printing out the errors
    for( auto& err : t)
    {
        if( err.code != 0 )
            out << std::endl << " ------- Error Trace --------";

        out << err;
    }

    return out;
}

std::ostream& operator<<(std::ostream& out, const error::ErrorHandler& t)
{
    out << t.readSilent();

    return out;
}

