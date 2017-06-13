#include "base_error/base_error.h"


// BaseError implementations

error::BaseError::BaseError( int errorCode )
    : errorCode_( errorCode )
{}

error::BaseError::BaseError( int errorCode, std::string errorMessage)
    : errorCode_( errorCode ),
      errorMessage_( errorMessage )
{
    this->hasMessage_ = true;
}

error::BaseError::BaseError( int errorCode, std::string errorMessage, ros::Time timeStamp)
    : errorCode_( errorCode ),
      errorMessage_( errorMessage ),
      timeStamp_( timeStamp )
{
    this->hasMessage_ = true;
    this->hasTimeStamp_ = true;
}

std::string error::BaseError::getErrorMessage()
{
    return this->errorMessage_;
}

int error::BaseError::getErrorCode()
{
    return this->errorCode_;
}

ros::Time error::BaseError::getTimeStamp()
{
    return this->timeStamp_;
}

bool error::BaseError::hasMessage()
{
    return this->hasMessage_;
}

bool error::BaseError::hasTimeStamp()
{
    return this->hasTimeStamp_;
}


// ErrorHandler implementations

void error::ErrorHandler::append( error::ErrorStack errorStack )
{
    this->errorStack_.insert(std::end(this->errorStack_), std::begin(errorStack), std::end(errorStack));
}

error::ErrorStack error::ErrorHandler::read()
{
    return this->errorStack_;
}

error::ErrorStack error::ErrorHandler::readAndClear()
{
    // Create a copy of the errorStack_
    error::ErrorStack errorStackCopy( this->errorStack_ );

    // Clear the errorStack_
    this->errorStack_.clear();

    // Return the copy
    return errorStackCopy;
}
