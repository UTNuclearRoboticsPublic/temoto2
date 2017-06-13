#ifndef BASE_ERROR_H
#define BASE_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"

class BaseError
{
public:

    BaseError( int errorCode );

    BaseError( int errorCode, std::string errorMessage);

    BaseError( int errorCode, std::string errorMessage, ros::Time timeStamp);

    std::string getErrorMessage();

    int getErrorCode();

    ros::Time getTimeStamp();

    bool hasMessage();

    bool hasTimeStamp();

private:

    std::string errorMessage_;

    int errorCode_;

    ros::Time timeStamp_;

    bool hasMessage_;

    bool hasTimeStamp_;
};

typedef std::vector <BaseError> errorStack;

#endif
