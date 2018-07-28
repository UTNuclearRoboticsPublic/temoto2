#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "temoto_error/temoto_error.h"
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "TTP/task_descriptor.h"
#include "TTP/task_tree.h"
#include "TTP/base_task/base_task.h"
#include "TTP/language_processors/meta/meta_lp.h"

#include "temoto_2/StopTask.h"
#include "temoto_2/IndexTasks.h"
#include "temoto_2/StopTaskMsg.h"
#include "std_msgs/String.h"

#include "tbb/flow_graph.h"
#include <class_loader/multi_library_class_loader.h>
#include <boost/any.hpp>
#include "boost/filesystem.hpp"
#include <exception>
#include <cstdio>
#include <thread>
#include <future>

namespace TTP
{

class TaskManager : public BaseSubsystem
{
public:


  TaskManager( std::string subsystem_name
             , error::Subsystem subsystem_code
             , bool nlp_enabled
             , std::string ai_libs_path = ""
             , std::string chatter_topic = "");

  TaskManager( BaseSubsystem* b
             , bool nlp_enabled
             , std::string ai_libs_path = ""
             , std::string chatter_topic = "");

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

  void executeSFTThreaded(TaskTree sft);

  void executeSFT (TaskTree sft);

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

  const std::string description_file_ = "descriptor.xml";

  bool action_executioner_busy_ = false;

  bool nlp_enabled_;


  ros::Timer thread_joining_timer_;

  std::vector<std::thread> flow_graph_threads_;

  std::vector<std::future<void>> flow_graph_futures_;

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
   * @brief tasks_indexed_
   */
  std::vector <TaskDescriptor> tasks_indexed_;

  /**
   * @brief class_loader_
   */
  class_loader::MultiLibraryClassLoader* class_loader_;

  std::map<std::string, boost::shared_ptr<class_loader::ClassLoader>> class_loaders_;

  void initCore(std::string ai_libs_path, std::string chatter_topic);

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

  /**
   * @brief threadJoiningTimerCallback
   * @param event
   */
  void threadJoiningTimerCallback(const ros::TimerEvent &e);

};

}// END of TTP namespace

#endif
