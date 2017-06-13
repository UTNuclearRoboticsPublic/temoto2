/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *    This is the base task that every task has to inherit and implement
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TASK_H
#define TASK_H

#include <string>
#include <boost/any.hpp>
#include "ros/ros.h"

class Task
{
public:

    /**
     * @brief Start the task
     * @return
     */
    virtual int startTask() = 0;

    /**
     * @brief startTask
     * @param subtaskNr Number of the subtask to be executed
     * @param arguments arguments Start the task by passing in arguments. boost::any is used as a
     * generic way ( imo better than void*) for not caring about the argument types
     * @return
     */
    virtual int startTask( int subtaskNr, std::vector<boost::any> arguments ) = 0;

    /**
     * @brief pauseTask
     * @return
     */
    virtual int pauseTask() = 0;

    /**
     * @brief stopTask
     * @return
     */
    virtual int stopTask() = 0;

    /**
     * @brief Receive a short description of the task
     * @return
     */
    virtual std::string getDescription() = 0;

    /**
     * @brief getStatus
     * @return
     */
    virtual std::string getStatus() = 0;

    /**
     * @brief getSolution
     * @param subtaskNr
     * @return
     */
    virtual std::vector<boost::any> getSolution( int subtaskNr) = 0;

    /**
     * @brief ~Task Implemented virtual constructor. If it would not be implemented,
     * a magical undefined .so reference error will appear if the task is destructed
     */
    virtual ~Task(){};

protected:

    std::string description;
};


#endif
