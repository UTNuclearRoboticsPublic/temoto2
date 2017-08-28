#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <class_loader/multi_library_class_loader.h>
#include <thread>
#include <boost/any.hpp>
#include "boost/filesystem.hpp"
#include <exception>

#include "core/common.h"
#include "core/task_handler/task_info.h"
#include "core/task_handler/running_task.h"
#include "core/task_handler/description_processor.h"
#include "base_task/task.h"
#include "temoto_2/stopTask.h"
#include "temoto_2/indexTasks.h"


class TaskHandler
{
public:

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    /**
     * @brief TaskHandler
     * @param system_prefix
     */
    TaskHandler(std::string system_prefix);

    /**
     * @brief findTask
     * @param task_to_find
     * @param tasks
     * @return
     */
    std::vector <TaskInfo> findTask(std::string task_to_find,  const std::vector <TaskInfo>& tasks);

    /**
     * @brief findTaskLocal
     * @param task_to_find
     * @return
     */
    std::vector <TaskInfo> findTaskLocal(std::string task_to_find);

    /**
     * @brief findTaskRunning
     * @param task_to_find
     * @return
     */
    //std::vector <TaskInfo> findTaskRunning(std::string task_to_find);

    /**
     * @brief findTask
     * @param task_to_find
     * @param base_path
     * @param search_depth
     * @return
     */
    std::vector <TaskInfo> findTaskFilesys(std::string task_to_find, boost::filesystem::directory_entry base_path, int search_depth);

    /**
     * @brief indexTasks
     * @param base_path
     * @param search_depth
     */
    void indexTasks (boost::filesystem::directory_entry base_path, int search_depth);

    /**
     * @brief getIndexedTasks
     * @return
     */
    std::vector <TaskInfo>* getIndexedTasks();

    /**
     * @brief executeTask
     * @param task_info
     * @param arguments
     * @return
     */
    bool executeTask(TaskInfo task_info, std::vector<boost::any> arguments);

    /**
     * @brief loadTask
     * @param task
     * @return
     */
    bool loadTask(RunningTask& task);

    /**
     * @brief instantiateTask
     * @param task
     * @return
     */
    bool instantiateTask(RunningTask& task);

    /**
     * @brief startTask
     * @param task
     * @return
     */
    bool startTask(RunningTask& task);

    /**
     * @brief startTask
     * @param task
     * @param arguments
     * @return
     */
    bool startTask(RunningTask& task, std::vector<boost::any> arguments);

    /**
     * @brief startTaskThread
     * @param task
     * @return
     */
    bool startTaskThread(RunningTask& task, std::vector<boost::any> arguments);

    /**
     * @brief Stops a task specified by the task_name
     * @param task_name
     * @return
     */
    bool stopTask(std::string task_name);

    /**
     * @brief unloadTaskLib
     * @param path_to_lib
     * @return
     */
    bool unloadTaskLib(std::string path_to_lib);

private:

    const std::string class_name_ = "TaskHandler";

    const std::string description_file_ = "description.xml";

    /**
     * @brief n_
     */
    ros::NodeHandle n_;

    // ros::ServiceServer startTaskServer_;
    ros::ServiceServer stop_task_server_;

    /**
     * @brief index_tasks_server_
     */
    ros::ServiceServer index_tasks_server_;

    /**
     * @brief system_prefix_
     */
    std::string system_prefix_;

    /**
     * @brief runningTasks_
     */
    std::vector <RunningTask> running_tasks_;

    /**
     * @brief tasks_indexed_
     */
    std::vector <TaskInfo>* tasks_indexed_;

    /**
     * @brief class_loader_
     */
    class_loader::MultiLibraryClassLoader* class_loader_;

    /**
     * @brief langProcessor_
     */
    //LanguageProcessor langProcessor_;

/*
    bool startTaskCallback (temoto_2::startTask::Request& req,
                            temoto_2::startTask::Response& res);
*/
    bool stopTaskCallback (temoto_2::stopTask::Request& req,
                           temoto_2::stopTask::Response& res);

    bool indexTasksCallback (temoto_2::indexTasks::Request& req,
                             temoto_2::indexTasks::Response& res);
};

#endif
