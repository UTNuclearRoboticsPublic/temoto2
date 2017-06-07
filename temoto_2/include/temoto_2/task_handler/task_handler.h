#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include "temoto_2/task_handler/common.h"
#include "temoto_2/task_handler/task_info.h"
#include "temoto_2/task_handler/description_processor.h"
#include "temoto_2/language_processor/language_processor.h"
#include "temoto_2/base_task/task.h"

#include <class_loader/multi_library_class_loader.h>
#include <map>
#include <string>
#include <vector>
#include <boost/any.hpp>
#include "boost/filesystem.hpp"

class TaskHandler
{
public:

    /**
     * @brief Task_handler
     */
    TaskHandler(class_loader::MultiLibraryClassLoader * loader);

    /**
     * @brief Finds and returns list of available tasks based on taskToFind criteria
     * @param taskToFind String that contains the type of the task, if specified as "any" then all task descriptions are loaded in
     * @return Returns a vector containing task description classes
     */
    std::vector <TaskInfo> findTask(std::string taskToFind);

    /**
     * @brief findTask
     * @param taskToFind
     * @param basePath
     * @param searchDepth
     * @return
     */
    std::vector <TaskInfo> findTask(std::string taskToFind, boost::filesystem::directory_entry basePath, int searchDepth);

    /**
     * @brief Loads in a task .so file and returns the name of the class
     * @param Full path to the task
     * @return Returns the name of the class. If same name already exists, an unique name is returned
     */
    std::string loadTask(std::string pathToLib);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    int instantiateTask(std::string taskName);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    int startTask(std::string taskName);

    /**
     * @brief startTask
     * @param taskName
     * @return
     */
    int startTask(std::string taskName, std::vector<boost::any> arguments);

    /**
     * @brief Stops a task specified by the taskName
     * @param taskName
     * @return
     */
    int stopTask(std::string taskName);

    /**
     * @brief unloadTaskLib
     * @param taskName
     * @return
     */
    int unloadTaskLib(std::string pathToLib);


private:
    /**
     * @brief runningTasks_
     */
    std::map < std::string, boost::shared_ptr<Task> > runningTasks_;

    /**
     * @brief taskAddresses_
     */
    std::vector <TaskAddress> taskAddresses_;

    /**
     * @brief taskToArgBook_
     */
    TaskToArgBook taskToArgBook_;

    /**
     * @brief loader_
     */
    class_loader::MultiLibraryClassLoader * loader_;

    /**
     * @brief descProcessor_
     */
    DescriptionProcessor descProcessor_;

    /**
     * @brief langProcessor_
     */
    LanguageProcessor langProcessor_;

    const std::string descriptionFile = "description.xml";
};

#endif
