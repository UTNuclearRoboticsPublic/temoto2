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


struct RunningTask
{
    std::string taskName;
    std::string taskClassName;
    boost::shared_ptr<Task> taskPointer;
};

class TaskHandler
{
public:

    /**
     * @brief errorHandler_
     */
    error::ErrorHandler errorHandler_;

    /**
     * @brief Task_handler
     */
    TaskHandler(std::string system_prefix, class_loader::MultiLibraryClassLoader * loader);

    /**
     * @brief findTask
     * @param taskToFind
     * @param tasks
     * @return
     */
    std::vector <TaskInfo> findTask(std::string taskToFind,  std::vector <TaskInfo>& tasks);

    /**
     * @brief findTaskLocal
     * @param taskToFind
     * @return
     */
    std::vector <TaskInfo> findTaskLocal(std::string taskToFind);

    /**
     * @brief findTaskRunning
     * @param taskToFind
     * @return
     */
    std::vector <TaskInfo> findTaskRunning(std::string taskToFind);

    /**
     * @brief findTask
     * @param taskToFind
     * @param basePath
     * @param searchDepth
     * @return
     */
    std::vector <TaskInfo> findTaskFilesys(std::string taskToFind, boost::filesystem::directory_entry basePath, int searchDepth);

    /**
     * @brief indexTasks
     * @param basePath
     * @param searchDepth
     */
    void indexTasks (boost::filesystem::directory_entry basePath, int searchDepth);

    /**
     * @brief getIndexedTasks
     * @return
     */
    std::vector <TaskInfo> getIndexedTasks();

    /**
     * @brief loadTask
     * @param taskName
     * @return
     */
    bool loadTask(TaskInfo& task);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    bool instantiateTask(TaskInfo& task);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    bool startTask(TaskInfo& task);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    bool startTask(TaskInfo& task, std::vector<boost::any> arguments);

    /**
     * @brief Stops a task specified by the taskName
     * @param taskName
     * @return
     */
    bool stopTask(std::string taskName);

    /**
     * @brief unloadTaskLib
     * @param taskName
     * @return
     */
    bool unloadTaskLib(std::string pathToLib);


private:

    ros::NodeHandle n_;

    // ros::ServiceServer startTaskServer_;
    ros::ServiceServer stop_task_server_;

    /**
     * @brief system_prefix_
     */
    std::string system_prefix_;

    /**
     * @brief runningTasks_
     */
    std::vector <TaskInfo> running_tasks_;

    /**
     * @brief tasksIndexed_
     */
    std::vector <TaskInfo> tasksIndexed_;

    /**
     * @brief loader_
     */
    class_loader::MultiLibraryClassLoader * loader_;

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
};

#endif
