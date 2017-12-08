#include "base_error/base_error.h"

/* * * * * * * * * * * * * * * * * * *
 *
 *    ErrorStackUtil implementations
 *
 * * * * * * * * * * * * * * * * * * */

/*
 * Constructor
 */
error::ErrorStackUtil::ErrorStackUtil( int code,
                                       error::Subsystem subsystem,
                                       error::Urgency urgency,
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
error::ErrorStackUtil::ErrorStackUtil( int code,
                                       error::Subsystem subsystem,
                                       error::Urgency urgency,
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
error::ErrorStack error::ErrorStackUtil::getStack()
{
    return this->error_stack_;
}

/*
 * Push
 */
void error::ErrorStackUtil::push( int code,
                                  error::Subsystem subsystem,
                                  error::Urgency urgency,
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
void error::ErrorStackUtil::push ( temoto_2::BaseError base_error )
{
    this->error_stack_.push_back ( base_error );
}

/*
 * Initialize Base Error
 */
temoto_2::BaseError error::ErrorStackUtil::initBaseError( int code,
                                                          error::Subsystem subsystem,
                                                          error::Urgency urgency,
                                                          std::string message,
                                                          ros::Time stamp)
{
    temoto_2::BaseError error_msg;
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
void error::ErrorStackUtil::forward ( std::string from_where )
{
    temoto_2::BaseError fwd_err = this->error_stack_.back();
    fwd_err.code = 0;
    fwd_err.message = from_where + " FORWARDING";

    this->push( fwd_err );
}


/* * * * * * * * * * * * * * * *
 *
 *    ErrorHandler implementations
 *
 * * * * * * * * * * * * * * * */

error::ErrorHandler::ErrorHandler()
{
    error_publisher_ = n_.advertise<temoto_2::ErrorStack>("temoto_error_messages", 100);
}

bool error::ErrorHandler::gotUnreadErrors() const
{
    return this->newErrors_;
}

void error::ErrorHandler::append( error::ErrorStack errorStack )
{
    // Set the "newErrors" flag and append
    this->newErrors_ = true;
    this->errorStack_.insert(std::end(this->errorStack_), std::begin(errorStack), std::end(errorStack));

    // Create an ErrorStack message and publish it
    temoto_2::ErrorStack error_stack_msg;
    error_stack_msg.ErrorStack = errorStack;

    error_publisher_.publish( error_stack_msg );
}

void error::ErrorHandler::append( error::ErrorStackUtil err_stck_util )
{
    append( err_stck_util.getStack() );
}

error::ErrorStack error::ErrorHandler::read()
{
    // Set the "newErrors" flag and return the stack
    this->newErrors_ = false;
    return this->errorStack_;
}

error::ErrorStack error::ErrorHandler::readSilent() const
{
    return this->errorStack_;
}

error::ErrorStack error::ErrorHandler::readAndClear()
{
    // Set the "newErrors" flag
    this->newErrors_ = false;

    // Create a copy of the errorStack_
    error::ErrorStack errorStackCopy( this->errorStack_ );

    // Clear the errorStack_
    this->errorStack_.clear();

    // Return the copy
    return errorStackCopy;
}


std::ostream& operator<<(std::ostream& out, const temoto_2::BaseError& t)
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

    out << RED << "* message: " << t.message << RESET << std::endl;
    out << "* timestamp: " << t.stamp  << std::endl;

    return out;
}

std::ostream& operator<<(std::ostream& out, const error::ErrorStack& t)
{
    out << std::endl;

    // Start printing out the errors
    for( auto err : t)
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
