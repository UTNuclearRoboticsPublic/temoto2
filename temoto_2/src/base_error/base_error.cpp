#include "base_error/base_error.h"


BaseError::BaseError( int errorCode )
    : errorCode_( errorCode )
{}

BaseError::BaseError( int errorCode, std::string errorMessage)
    : errorCode_( errorCode ),
      errorMessage_( errorMessage )
{
    this->hasMessage_ = true;
}

BaseError::BaseError( int errorCode, std::string errorMessage, ros::Time timeStamp)
    : errorCode_( errorCode ),
      errorMessage_( errorMessage ),
      timeStamp_( timeStamp )
{
    this->hasMessage_ = true;
    this->hasTimeStamp_ = true;
}

std::string BaseError::getErrorMessage()
{
    return this->errorMessage_;
}

int BaseError::getErrorCode()
{
    return this->errorCode_;
}

ros::Time BaseError::getTimeStamp()
{
    return this->timeStamp_;
}

bool hasMessage()
{
    return this->hasMessage_;
}

bool hasTimeStamp()
{
    return this->hasTimeStamp_;
}
