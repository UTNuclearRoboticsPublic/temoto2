/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *    This is the base task that every task has to inherit and implement
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TASK_H
#define TASK_H

#include <string>
#include <boost/any.hpp>
#include "base_error/base_error.h"
#include "common/temoto_id.h"
#include "temoto_2/StopTaskMsg.h"
#include "ros/ros.h"

class Task
{
friend class TaskHandler;

public:

    /**
     * @brief Start the task
     * @return
     */
    virtual bool startTask(){return true;};

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
	virtual std::string getStatus()
	{
		return "healthy";
	}


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

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

protected:

    std::string description;
    bool stop_task_ = false;

private:

    /**
     * @brief task_n_
     */
    ros::NodeHandle task_nodehandle_;

    /**
     * @brief task_id_
     */
    TemotoID::ID task_id_ = TemotoID::UNASSIGNED_ID;

    /**
     * @brief startTaskAutojoin
     * @param subtaskNr
     * @param arguments
     */
    void startTaskAutostop( int subtaskNr, std::vector<boost::any> arguments )
    {
        // Start the task
        startTask( subtaskNr, arguments );

        // After the task has finished, let the Task Handler know that the task is finished
        ros::Publisher stop_task_pub = task_nodehandle_.advertise<temoto_2::StopTaskMsg>("temoto/stop_task", 10);

        // Wait for the publisher to come up
        while( stop_task_pub == nullptr){}

        // Send the stop message
        temoto_2::StopTaskMsg stop_task_msg;
        stop_task_msg.task_id = static_cast<int>(task_id_);
        stop_task_pub.publish( stop_task_msg );
    }

    /**
     * @brief setID
     * @param task_id
     */
    void setID( TemotoID::ID task_id )
    {
        task_id_ = task_id;
    }

    /**
     * @brief getID
     * @return
     */
    TemotoID::ID getID()
    {
        return task_id_;
    }
};
#endif
