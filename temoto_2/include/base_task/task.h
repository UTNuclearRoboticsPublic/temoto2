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
    virtual bool startTask() = 0;

    /**
     * @brief startTask
     * @param subtaskNr Number of the subtask to be executed
     * @param arguments arguments Start the task by passing in arguments. boost::any is used as a
     * generic way ( imo better than void*) for not caring about the argument types
     * @return
     */
    virtual bool startTask( int subtaskNr, std::vector<boost::any> arguments ) = 0;

    /**
     * @brief pauseTask
     * @return
     */
    bool pauseTask();

    /**
     * @brief stopTask
     * @return
     */
    bool stopTask()
    {
        stop_task_ = true;
        return 0;
    }

    /**
     * @brief Receive a short description of the task
     * @return
     */
    std::string getDescription();

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
    bool stop_task_ = false;
};


#endif
