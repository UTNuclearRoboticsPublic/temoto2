#include "base_error/base_error.h"


// BaseError implementations

error::BaseError::BaseError( int code,
                             Subsystem subsystem,
                             Urgency urgency )
    : code_( code ),
      subsystem_( subsystem ),
      urgency_( urgency )
{}

error::BaseError::BaseError( int code,
                             Subsystem subsystem,
                             Urgency urgency,
                             std::string message)
    : code_( code ),
      subsystem_( subsystem ),
      urgency_( urgency ),
      message_( message )
{
    this->hasMessage_ = true;
}

error::BaseError::BaseError( int code,
                             Subsystem subsystem,
                             Urgency urgency,
                             std::string message,
                             ros::Time timeStamp)
    : code_( code ),
      subsystem_( subsystem ),
      urgency_( urgency ),
      message_( message ),
      timeStamp_( timeStamp )
{
    this->hasMessage_ = true;
    this->hasTimeStamp_ = true;
}

int error::BaseError::getCode() const
{
    return this->code_;
}

error::Subsystem error::BaseError::getSubsystem() const
{
    return this->subsystem_;
}

error::Urgency error::BaseError::getUrgency() const
{
    return this->urgency_;
}

std::string error::BaseError::getMessage() const
{
    return this->message_;
}

ros::Time error::BaseError::getTimeStamp() const
{
    return this->timeStamp_;
}

bool error::BaseError::hasMessage() const
{
    return this->hasMessage_;
}

bool error::BaseError::hasTimeStamp() const
{
    return this->hasTimeStamp_;
}


// ErrorHandler implementations

bool error::ErrorHandler::gotUnreadErrors() const
{
    return this->newErrors_;
}

void error::ErrorHandler::append( error::ErrorStack errorStack )
{
    // Set the "newErrors" flag and append
    this->newErrors_ = true;
    this->errorStack_.insert(std::end(this->errorStack_), std::begin(errorStack), std::end(errorStack));
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

std::ostream& operator<<(std::ostream& out, const error::BaseError& t)
{
    out << std::endl;
    out << "* code: " << t.getCode() << std::endl;
    out << "* subsystem: " << static_cast<int>(t.getSubsystem()) << std::endl;
    out << "* urgency: " ;

    error::Urgency urg = t.getUrgency();
    if ( urg == error::Urgency::LOW )
        out << "LOW" << std::endl;

    else if ( urg == error::Urgency::MEDIUM )
        out << "MEDIUM" << std::endl;

    else if ( urg == error::Urgency::HIGH )
        out << "HIGH" << std::endl;

    out << "* message: " << t.getMessage() << std::endl;
    out << "* timestamp: " << t.getTimeStamp() << std::endl;

    return out;
}

std::ostream& operator<<(std::ostream& out, const error::ErrorHandler& t)
{
    out << std::endl;

    // Start printing out the errors
    for( error::BaseError err : t.readSilent())
    {
        if( err.getCode() != 0 )
        {
            out << std::endl << " ------- Error Trace --------";
        }

        out << err;
    }

    return out;
}
