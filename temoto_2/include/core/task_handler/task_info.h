#ifndef TASK_INFO_H
#define TASK_INFO_H

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <boost/any.hpp>
#include <thread>

//#include "base_task/task.h"

class TaskInfo;

typedef std::pair <std::string, std::vector<std::string>> ArgWithValues;
typedef std::vector < std::vector<ArgWithValues> > ParamList;
typedef std::vector< std::pair<TaskInfo, std::vector<boost::any>> > TaskList;

/**
 * @brief The TaskInfo class contains informaton about a specific task
 */
class TaskInfo
{
    friend class DescriptionProcessor;
    friend class TaskHandler;

public:

    /**
     * @brief Returns the path to the task
     * @return
     */
    std::string getPath() const;

    /**
     * @brief getLibPath
     * @return
     */
    std::string getLibPath() const;

    /**
     * @brief Returns the name of the task
     * @return
     */
    std::string getName() const;

    /**
     * @brief getClassName
     * @return
     */
    std::string getClassName() const;

    /**
     * @brief getPackageNAme
     * @return
     */
    std::string getPackageName() const;

    /**
     * @brief Returns the list of accepted input arguments
     * @return
     */
    ParamList getArgs() const;

    /**
     * @brief Returns the list possible return combinations (vector of boost::any)
     * @return
     */
    ParamList getReturn() const;

private:

    std::string name_;

    std::string class_name_;

    std::string packageName_;

    std::string path_;

    std::string libPath_;

    ParamList args_;

    ParamList return_;

   //boost::shared_ptr<Task> task_pointer_;
};

/**
 * @brief Places the information about a task into output stream
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const TaskInfo& t);

#endif
