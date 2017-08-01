#ifndef TASK_INFO_H
#define TASK_INFO_H

#include "core/common.h"

/**
 * @brief The TaskInfo class contains informaton about a specific task
 */
class TaskInfo
{
    friend class DescriptionProcessor;

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

    std::string packageName_;

    std::string path_;

    std::string libPath_;

    ParamList args_;

    ParamList return_;
};

/**
 * @brief Places the information about a task into output stream
 * @param out
 * @param t
 * @return
 */
std::ostream& operator<<(std::ostream& out, const TaskInfo& t);

#endif
