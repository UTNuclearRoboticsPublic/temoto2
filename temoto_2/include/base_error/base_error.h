#ifndef BASE_ERROR_H
#define BASE_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"

namespace error
{
/**
 * @brief Enum that stores the subsystem codes
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
 * @brief Enum that stores the urgency leveles
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

    /**
     * @brief BaseError
     * @param code
     * @param subsystem
     * @param urgency
     */
    BaseError( int code,
               Subsystem subsystem,
               Urgency urgency );

    /**
     * @brief BaseError
     * @param code
     * @param subsystem
     * @param urgency
     * @param message
     */
    BaseError( int code,
               Subsystem subsystem,
               Urgency urgency,
               std::string message );

    /**
     * @brief BaseError
     * @param code
     * @param subsystem
     * @param urgency
     * @param message
     * @param timeStamp
     */
    BaseError( int code,
               Subsystem subsystem,
               Urgency urgency,
               std::string message,
               ros::Time timeStamp );

    /**
     * @brief getCode
     * @return
     */
    int getCode() const;

    /**
     * @brief getSubsystem
     * @return
     */
    Subsystem getSubsystem() const;

    /**
     * @brief getUrgency
     * @return
     */
    Urgency getUrgency() const;

    /**
     * @brief getMessage
     * @return
     */
    std::string getMessage() const;

    /**
     * @brief getTimeStamp
     * @return
     */
    ros::Time getTimeStamp() const;

    /**
     * @brief hasMessage
     * @return
     */
    bool hasMessage() const;

    /**
     * @brief hasTimeStamp
     * @return
     */
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

    /**
     * @brief Appends the ErrorStack
     * @param errorStack
     */
    void append( ErrorStack errorStack );

    /**
     * @brief Returns the ErrorStack and sets the "newErrors" flag to false
     * @return
     */
    ErrorStack read();

    /**
     * @brief Returns the ErrorStack and does not change the "newErrors" flag
     * @return
     */
    ErrorStack readSilent() const;

    /**
     * @brief Returns the ErrorStack, sets the "newErrors" flag to false and clears the stack
     * @return
     */
    ErrorStack readAndClear();

    /**
     * @brief Returns "true" if there are any new unread errors
     * @return
     */
    bool gotUnreadErrors() const;

private:

    bool newErrors_ = false;

    ErrorStack errorStack_;
};

/**
 * @brief formatError
 * @param code
 * @param subsystem
 * @param urgency
 * @param message
 * @param timeStamp
 * @return
 */
ErrorStack formatError ( int code,
                         Subsystem subsystem,
                         Urgency urgency,
                         std::string message,
                         ros::Time timeStamp );


} // end of error namespace

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const error::BaseError& t);

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const error::ErrorHandler& t);

#endif
