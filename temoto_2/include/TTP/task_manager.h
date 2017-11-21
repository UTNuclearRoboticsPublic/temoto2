#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <class_loader/multi_library_class_loader.h>
#include <boost/any.hpp>
#include "boost/filesystem.hpp"
#include <exception>

#include "base_error/base_error.h"
#include "common/temoto_id.h"
#include "TTP/task_descriptor.h"
#include "TTP/task_tree.h"
#include "TTP/base_task/base_task.h"
#include "TTP/language_processors/meta/meta_lp.h"

#include "temoto_2/StopTask.h"
#include "temoto_2/IndexTasks.h"
#include "temoto_2/StopTaskMsg.h"
#include "std_msgs/String.h"

#include <cstdio>
#include "tbb/flow_graph.h"

namespace TTP
{

class TaskManager
{
public:

    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    /**
     * @brief TaskManager
     * @param system_prefix
     */
    TaskManager(std::string system_prefix);

    /**
     * @brief findTask
     * @param task_to_find
     * @param tasks
     * @return
     */
    std::vector <TaskDescriptor> findTask(std::string task_to_find,  const std::vector <TaskDescriptor>& tasks);

    /**
     * @brief findTaskLocal
     * @param task_to_find
     * @return
     */
    std::vector <TaskDescriptor> findTaskLocal(std::string task_to_find);

    /**
     * @brief findTask
     * @param task_to_find
     * @param base_path
     * @param search_depth
     * @return
     */
    std::vector <TaskDescriptor> findTaskFilesys(std::string task_to_find, boost::filesystem::directory_entry base_path, int search_depth);

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
    std::vector <TaskDescriptor>& getIndexedTasks();

    void connectTaskTree(TaskTreeNode& node, std::vector<Subject> parent_subjects, unsigned int depth = 0);

    void loadAndInitializeTaskTree(TaskTreeNode& node);

    void makeFlowGraph(TaskTreeNode& node, tbb::flow::graph& flow_graph);

    void connectFlowGraph(TaskTreeNode& node);

    void executeVerbalInstruction (std::string& verbal_instruction);

    /**
     * @brief loadTask
     * @param task_descriptor
     */
    void loadTask(TaskDescriptor& task_descriptor);

    /**
     * @brief unloadTaskLib
     * @param task_descriptor
     */
    void unloadTaskLib(std::string path_to_lib);

    /**
     * @brief instantiateTask
     * @param node
     */
    void instantiateTask(TaskTreeNode& node);

    /**
     * @brief stopTask
     * @param action
     * @param what
     */
    void stopTask(std::string action = "", std::string what = "");



private:

    const std::string class_name_ = "TaskManager";

    const std::string log_group_ = "temoto_core";

    const std::string description_file_ = "descriptor.xml";

    bool action_executioner_busy_ = false;

    std::vector<std::pair<boost::shared_ptr<TaskDescriptor>, boost::shared_ptr<BaseTask>>> asynchronous_tasks_;

    std::vector<std::string> synchronous_task_libs_;

    TTP::MetaLP* language_processor_;

    ros::Subscriber human_chatter_subscriber_;

    /**
     * @brief n_
     */
    ros::NodeHandle nh_;

    /**
     * @brief id_manager_
     */
    TemotoID::IDManager id_manager_;


    ros::ServiceServer stop_task_server_;

    /**
     * @brief index_tasks_subscriber_
     */
    ros::Subscriber index_tasks_subscriber_;

    /**
     * @brief join_task_server_
     */
    ros::Subscriber stop_task_subscriber_;

    /**
     * @brief system_prefix_
     */
    std::string system_prefix_;

    /**
     * @brief tasks_indexed_
     */
    std::vector <TaskDescriptor> tasks_indexed_;

    /**
     * @brief class_loader_
     */
    class_loader::MultiLibraryClassLoader* class_loader_;

    std::map<std::string, class_loader::ClassLoader*> class_loaders_;

    /**
     * @brief langProcessor_
     */


    /**
     * @brief stopTaskCallback
     * @param req
     * @param res
     * @return
     */
    bool stopTaskCallback (temoto_2::StopTask::Request& req,
                           temoto_2::StopTask::Response& res);

    /**
     * @brief indexTasksCallback
     * @param index_msg
     * @return
     */
    void indexTasksCallback (temoto_2::IndexTasks index_msg);

    /**
     * @brief humanChatterCb
     * @param chat
     */
    void humanChatterCb (std_msgs::String chat);

};

}// END of TTP namespace

#endif
