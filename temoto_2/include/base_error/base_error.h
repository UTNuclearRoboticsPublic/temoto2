#ifndef BASE_ERROR_H
#define BASE_ERROR_H

#include <string>
#include <vector>
#include "ros/ros.h"
#include "temoto_2/BaseError.h"
#include "temoto_2/ErrorStack.h"
#include "common/console_colors.h"

namespace error
{

const int FORWARDING_CODE = 0;

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
    OUTPUT_MANAGER,
    TASK
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
 * @brief ErrorStack
 */
typedef std::vector <temoto_2::BaseError> ErrorStack;


/**
 * @brief The ErrorStackUtil class
 */
class ErrorStackUtil
{
public:

    /**
     * @brief ErrorStackUtil
     * @param code
     * @param subsystem
     * @param urgency
     * @param message
     * @param timeStamp
     */
    ErrorStackUtil ( int code,
                       Subsystem subsystem,
                       Urgency urgency,
                       std::string message,
                       ros::Time timeStamp = ros::Time::now());

    /**
     * @brief getStack
     * @return
     */
    ErrorStack getStack ();

    /**
     * @brief push
     * @param code
     * @param subsystem
     * @param urgency
     * @param message
     * @param timeStamp
     */
    void push ( int code,
                Subsystem subsystem,
                Urgency urgency,
                std::string message,
                ros::Time timeStamp = ros::Time::now());

    /**
     * @brief apush
     * @param base_error
     */
    void push ( temoto_2::BaseError base_error );

    /**
     * @brief forward
     * @param message
     */
    void forward ( std::string from_where );

private:

    /**
     * @brief error_stack_
     */
    ErrorStack error_stack_;

    /**
     * @brief initBaseError
     * @param code
     * @param subsystem
     * @param urgency
     * @param message
     * @param stamp
     * @return
     */
    temoto_2::BaseError initBaseError ( int code,
                                        Subsystem subsystem,
                                        Urgency urgency,
                                        std::string message,
                                        ros::Time stamp = ros::Time::now());
};


/**
 * @brief The ErrorHandler class
 */
class ErrorHandler
{
public:

    ErrorHandler();

    /**
     * @brief Appends the ErrorStack
     * @param errorStack
     */
    void append( ErrorStack errorStack );

    /**
     * @brief append
     * @param err_stk_util
     */
    void append( ErrorStackUtil err_stck_util );

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

    ros::NodeHandle n_;

    ros::Publisher error_publisher_;

    bool newErrors_ = false;

    ErrorStack errorStack_;
};


} // end of error namespace

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const temoto_2::BaseError& t);

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const error::ErrorStack& t);

/**
 * @brief operator <<
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const error::ErrorHandler& t);

#endif
