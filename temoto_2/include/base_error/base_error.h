#ifndef BASE_ERROR_H
#define BASE_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"

namespace error
{
/**
     * @brief The Subsystem enum
     */
    enum class Subsystem : int
    {
        CORE,
        CONTEXT_CENTER,
        HEALTH_MONITOR,
        SENSOR_MANAGER,
        ROBOT_MANAGER,
        OUTPUT_MANAGER
    };

    /**
     * @brief The Urgency enum
     */
    enum class Urgency : int
    {
        LOW,        // Does not affect the performance of the system directly
        MEDIUM,     // Does not affect the performance of the system directly, but needs to be resolved
        HIGH        // Affects the performance of the system, needs to be resolved immediately
    };



    /**
     * @brief The BaseError class
     */
    class BaseError
    {
    public:

        BaseError( int code,
                   Subsystem subsystem,
                   Urgency urgency );

        BaseError( int code,
                   Subsystem subsystem,
                   Urgency urgency,
                   std::string message );

        BaseError( int code,
                   Subsystem subsystem,
                   Urgency urgency,
                   std::string message,
                   ros::Time timeStamp );

        int getCode() const;

        Subsystem getSubsystem() const;

        Urgency getUrgency() const;

        std::string getMessage() const;

        ros::Time getTimeStamp() const;

        bool hasMessage() const;

        bool hasTimeStamp() const;

    private:

        int code_;

        Subsystem subsystem_;

        Urgency urgency_;

        std::string message_;

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

        ErrorStack readSilent() const;

        ErrorStack readAndClear();

        bool gotUnreadErrors() const;

    private:

        bool newErrors_ = false;

        ErrorStack errorStack_;
    };
}

std::ostream& operator<<(std::ostream& out, const error::BaseError& t);

std::ostream& operator<<(std::ostream& out, const error::ErrorHandler& t);

#endif
