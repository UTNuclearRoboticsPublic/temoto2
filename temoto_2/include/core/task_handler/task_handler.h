#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <class_loader/multi_library_class_loader.h>
#include <boost/any.hpp>
#include "boost/filesystem.hpp"
#include <exception>

#include "core/common.h"
#include "core/task_handler/task_info.h"
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
    std::vector <TaskInfo> findTaskRunning(std::string task_to_find);

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
     * @brief loadTask
     * @param task
     * @return
     */
    bool loadTask(TaskInfo& task);

    /**
     * @brief instantiateTask
     * @param task
     * @return
     */
    bool instantiateTask(TaskInfo& task);

    /**
     * @brief startTask
     * @param task
     * @return
     */
    bool startTask(TaskInfo& task);

    /**
     * @brief startTask
     * @param task
     * @param arguments
     * @return
     */
    bool startTask(TaskInfo& task, std::vector<boost::any> arguments);

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
    std::vector <TaskInfo> running_tasks_;

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

    const std::string descriptionFile = "description.xml";
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
