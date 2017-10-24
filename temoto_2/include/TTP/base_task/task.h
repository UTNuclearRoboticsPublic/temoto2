/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *    This is the base task that every task has to inherit and implement
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TASK_H
#define TASK_H

#include <string>
#include "TTP/task_descriptor.h"
#include <boost/any.hpp>
#include "base_error/base_error.h"
#include "common/temoto_id.h"
#include "temoto_2/StopTaskMsg.h"
#include "ros/ros.h"

namespace TTP
{

class Task
{
friend class TaskManager;

public:

    /**
     * @brief startTask
     * @param task_interface
     * @return
     */
    virtual bool startTask(TaskInterface task_interface) = 0;

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
     * @return
     */
    virtual std::vector<Subject> getSolution() = 0;

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
}// END of TTP namespace
#endif
