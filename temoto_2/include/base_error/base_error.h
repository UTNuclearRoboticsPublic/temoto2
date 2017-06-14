#ifndef BASE_ERROR_H
#define BASE_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"

namespace error
{
    /**
     * @brief The BaseError class
     */
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

    /**
     * @brief errorStack
     */
    typedef std::vector <BaseError> ErrorStack;


    /**
     * @brief The ErrorHandler class
     */
    class ErrorHandler
    {
    public:

        void append( ErrorStack errorStack );

        ErrorStack read();

        ErrorStack readAndClear();

        bool gotUnreadErrors();

    private:

        bool newErrors_ = false;

        ErrorStack errorStack_;
    };
}
#endif
