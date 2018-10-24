/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *     MASSIVE TODO: * CATCH ALL EXEPTIONS AND RETHROW AS
 *                     TEMOTO ERROR !!!
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "ros/ros.h"
#include "ros/package.h"

#include "common/tools.h"
#include "common/temoto_log_macros.h"
#include "TTP/task_manager.h"
#include "TTP/task_descriptor_processor.h"
#include "TTP/task_container.h"

#include <boost/filesystem/operations.hpp>
#include <algorithm>

namespace TTP
{

struct CandidateInterface
{
  CandidateInterface(TaskInterface inf, unsigned int score):inf_(inf), score_(score){}
  TaskInterface inf_;
  int score_;
};

struct CandidateTask
{
  CandidateTask(TaskDescriptor td, CandidateInterface ci):td_(td), ci_(ci){}
  TaskDescriptor td_;
  CandidateInterface ci_;
};


/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskManager::TaskManager( std::string subsystem_name
                        , error::Subsystem subsystem_code
                        , bool nlp_enabled
                        , std::string ai_libs_path
                        , std::string chatter_topic)
  : nlp_enabled_(nlp_enabled)
{
  try
  {
    // Get the name of this class
    class_name_ = __func__;

    // Initialize the base subsystem members
    subsystem_name_ = subsystem_name;
    subsystem_code_ = subsystem_code;
    log_group_ = "core";
    error_handler_ = error::ErrorHandler(subsystem_code_, "core");

    // Initialize the core
    initCore(ai_libs_path, chatter_topic);
  }

  catch (error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }
}

/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskManager::TaskManager( BaseSubsystem *b
                        , bool nlp_enabled
                        , std::string ai_libs_path
                        , std::string chatter_topic)
  : BaseSubsystem(*b)
  , nlp_enabled_(nlp_enabled)
{
  try
  {
    // Get the name of this class
    class_name_ = __func__;

    // Initialize the core
    initCore(ai_libs_path, chatter_topic);
  }

  catch (error::ErrorStack& e)
  {
    // Rethrow or do whatever
    // std::cout << e.getStack();
  }
}

void TaskManager::initCore(std::string ai_libs_path, std::string chatter_topic)
{
  // Construct the classloader
  class_loader_ = new class_loader::MultiLibraryClassLoader(true);

  // Create a base path for getting stuff like nlp model files and tasks
  std::string temoto_path = ros::package::getPath(ROS_PACKAGE_NAME);

  /*
   * If the language processor is enabled, then initialize
   * it by giving it the path to language model files
   */
  if (nlp_enabled_)
  {
    language_processor_ = new MetaLP( temoto_path + "/language_processors/meta/models/"
                                    , *this
                                    , common::getTemotoNamespace());

    // Subscribe to human chatter topic. This triggers the callback that processes text
    // messages and trys to find and execute tasks based on the text
    human_chatter_subscriber_ = nh_.subscribe(chatter_topic, 1, &TaskManager::humanChatterCb, this);
  }

  /*
   * Index (look recursivey for the tasks in the given folder up to specified depth)
   * the available tasks, otherwise the task handler would have no clue about the available
   * tasks. Later the indexing could be via indexing task.
   * TODO: Read the base path from the parameter server
   */
  std::cout << "Indexing the tasks ... " << std::flush;
  boost::filesystem::directory_entry dir;

  if (!ai_libs_path.empty())
  {
    dir = boost::filesystem::directory_entry(ai_libs_path);
  }

  else
  {
    dir = boost::filesystem::directory_entry(temoto_path + "/../temoto_actions/user_actions");
  }

  indexTasks(dir, 2);
  std::cout << "done\n";

  // Stop task server
  stop_task_server_ = nh_.advertiseService ("stop_task", &TaskManager::stopTaskCallback, this);

  // Task indexing subscriber
  index_tasks_subscriber_ = nh_.subscribe("index_tasks", 1, &TaskManager::indexTasksCallback, this);

  // Thread joining timer
  thread_joining_timer_ = nh_.createTimer(ros::Duration(1), &TaskManager::threadJoiningTimerCallback, this);
}

/* * * * * * * * *
 *  humanChatterCb
 * * * * * * * * */

void TaskManager::humanChatterCb (std_msgs::String chat)
{
  try
  {
    std::cout << BOLDWHITE << "Received: " << chat.data << RESET << std::endl << std::endl;
    executeVerbalInstruction (chat.data);
  }
  catch (error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }

  std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * \n\n";
}


/* * * * * * * * *
 *  EXECUTE VERBAL INSTRUCTION
 * * * * * * * * */

void TaskManager::executeVerbalInstruction (std::string& verbal_instruction)
{
  try
  {
    // First check if NLP is enabled, if not then throw an error
    if (!nlp_enabled_)
    {
      throw CREATE_ERROR(error::Code::NLP_DISABLED, "NLP cannot be used if its disabled.");
    }

    // Convert the verbal instruction into a incomplete semantic frame tree
    TaskTree sft = language_processor_->processText(std::move(verbal_instruction));

    // Execute the SFT
    executeSFTThreaded(std::move(sft));

  }
  catch(error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }

  return;
}


/* * * * * * * * *
 *  EXECUTE SFT THREADED
 * * * * * * * * */
void TaskManager::executeSFTThreaded(TaskTree sft)
{
  flow_graph_futures_.push_back(std::async(std::launch::async, &TaskManager::executeSFT, this, std::move(sft)));
}

/* * * * * * * * *
 *  EXECUTE SEMANTIC FRAME TREE
 * * * * * * * * */

void TaskManager::executeSFT(TaskTree sft)
{
  // Let others know that action execution engine is busy
  action_executioner_busy_ = true;

  try
  {
    TaskTree sft_new = std::move(sft);
    TaskTreeNode& root_node = sft_new.getRootNode();

    // Print out the semantic frame tree
    std::cout << "SFTree: " << sft_new;

    // Find connecting semantic frames
    TEMOTO_DEBUG_STREAM("Connecting the task tree");
    std::vector<TTP::Subject> empty_subs; // stupid hack
    connectTaskTree(root_node, empty_subs);

    // Print task tree task descriptors
    sft_new.printTaskDescriptors(root_node);

    // Load and initialize the tasks
    TEMOTO_DEBUG_STREAM("Loadng and initializing the tree");
    loadAndInitializeTaskTree(root_node);

    // Create a tbb flow graph
    TEMOTO_DEBUG_STREAM("Creating an empty flow graph object");
    tbb::flow::graph flow_graph;

    // Build flow graph nodes
    TEMOTO_DEBUG_STREAM("Building flow graph nodes based on SF tree nodes");
    makeFlowGraph(root_node, flow_graph); // TODO: better name would be populateFlowgraph

    // Connect flow graph nodes
    TEMOTO_DEBUG_STREAM("Connecting flow graph nodes");
    connectFlowGraph(root_node);

    // Start the flow graph
    TEMOTO_DEBUG_STREAM("Starting the flow graph");
    TTP::Subjects dummy_subjects; // stupid hack
    root_node.root_fgn_->try_put(dummy_subjects);
    flow_graph.wait_for_all();

    TEMOTO_DEBUG_STREAM("Finished executing the flow graph");

  }
  catch(error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }
  catch(...)
  {
    throw CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
  }

  // Let others know that action execution engine is not busy
  // TODO: in a multithreaded case this will fail
  action_executioner_busy_ = false;
}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskDescriptor> TaskManager::findTask(std::string task_to_find, const std::vector <TaskDescriptor>& tasks)
{
  std::vector <TaskDescriptor> tasks_found;
  for (auto& task : tasks)
  {
    if (task_to_find == task.getAction())
    {
      tasks_found.push_back(task);
    }
  }
  return tasks_found;
}


/* * * * * * * * *
 *  FIND TASK LOCAL
 * * * * * * * * */

std::vector <TaskDescriptor> TaskManager::findTaskLocal(std::string task_to_find)
{
  return findTask(task_to_find, tasks_indexed_);
}

/* * * * * * * * *
 *  FIND TASK FROM FILESYSTEM
 * * * * * * * * */

std::vector <TaskDescriptor> TaskManager::findTaskFilesys(std::string task_to_find,
                                                          boost::filesystem::directory_entry base_path,
                                                          int search_depth)
{
  boost::filesystem::path current_dir (base_path);
  boost::filesystem::directory_iterator end_itr;
  std::vector <TaskDescriptor> tasks_found;

  try
  {
    // Start looking the files inside current directory
    for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
    {

      // if its a directory and depth limit is not there yet, go inside it
      if ( boost::filesystem::is_directory(*itr) && (search_depth > 0) )
      {
        std::vector<TaskDescriptor> sub_tasks_found = findTaskFilesys( task_to_find, *itr, (search_depth - 1) );

        // Append the subtasks if not empty
        if ( !sub_tasks_found.empty() )
        {
          tasks_found.insert(std::end(tasks_found), std::begin(sub_tasks_found), std::end(sub_tasks_found));
        }
      }

      // if its a file and matches the desc file name, process the file
      else if ( boost::filesystem::is_regular_file(*itr) &&
                ((*itr).path().filename() == description_file_) )
      {
        try
        {
          /*
           * Create a description processor object and Get TaskDescriptor
           * I THINK THIS SHOULD NOT BE CREATED EVERY SINGLE TIME
           */
          boost::filesystem::path hackdir ((*itr)); //HACKATON
          TaskDescriptorProcessor tdp(hackdir.parent_path().string(), *this);
          tasks_found.push_back(tdp.getTaskDescriptor());
        }

        catch(error::ErrorStack& error_stack)
        {
          FORWARD_ERROR(error_stack);
        }
      }
    }
    return std::move(tasks_found);
  }
  catch (std::exception& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(error::Code::FIND_TASK_FAIL, e.what());
  }

  catch(...)
  {
    // Rethrow the exception
    throw CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
  }
}

/* * * * * * * * *
 *  INDEX TASKS
 * * * * * * * * */

void TaskManager::indexTasks (boost::filesystem::directory_entry base_path, int search_depth)
{
    try
    {
        TEMOTO_DEBUG_STREAM("Indexing the tasks");
        tasks_indexed_ = findTaskFilesys ("", base_path, search_depth);
        TEMOTO_DEBUG_STREAM("Found " << tasks_indexed_.size() << " tasks");

        // Unload synchronous task libraries
        TEMOTO_DEBUG_STREAM("Unloading synchronous action libraries");

        for (auto& task_lib : synchronous_task_libs_)
        {
            unloadTaskLib(task_lib);
        }

        // Clear the synchronous task libraries set
        synchronous_task_libs_.clear();
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
}


/* * * * * * * * *
 *  GET THE INDEX TASKS
 * * * * * * * * */

std::vector <TaskDescriptor>& TaskManager::getIndexedTasks()
{
    return tasks_indexed_;
}


/* * * * * * * * *
 *  FIND NONSTRICT MATCH
 * * * * * * * * */

unsigned int findNonstrictMatch (const std::vector<Subject>& complete_subjects,
                                 const std::vector<Subject>& incomplete_subjects)
{
    // create a copy of complete subjects
    std::vector<Subject> complete_subjects_c = complete_subjects;
    unsigned int score = 0;

    for (auto& i_subject : incomplete_subjects)
    {
        bool subject_match = false;
        for (unsigned int i=0; i<complete_subjects_c.size(); i++)
        {
            // Check for type match
            if (i_subject.type_ != complete_subjects_c[i].type_)
            {
                continue;
            }

            subject_match = true;
            score++;

            // The more data the complete subject has, the bigger the score
            score += complete_subjects_c[i].data_.size();
            complete_subjects_c.erase(complete_subjects_c.begin() + i);
            break;
        }
        if (subject_match != true)
        {
            return 0; // Return the score of zero
        }
    }
    return score;
}


/* * * * * * * * *
 *  FIND WORD MATCH + STRICT MATCH
 * * * * * * * * */

unsigned int findWordMatch (Subjects& required_subjects , Subjects& node_subjects)
{
    // create a copy of subs_2
    Subjects node_subjects_c = node_subjects;
    unsigned int score = 0;

    for (auto& r_sub : required_subjects)
    {
        bool subject_match = false;
        for (auto n_sub = node_subjects_c.begin();
             n_sub != node_subjects_c.end();
             ++n_sub)
        {
            // Check for type match
            if (r_sub.type_ != n_sub->type_)
            {
                //std::cout << "    subject types do not match\n";
                continue;
            }

            // Check words
            if (!r_sub.words_.empty())
            {
                if (n_sub->words_.empty())
                {
                    continue;
                }

                // Go through the list of possible required words
                bool word_match = false;
                for (auto& r_word : r_sub.words_)
                {
                    if (r_word == n_sub->words_[0])
                    {
                        // Required word match is awarded with a higher score than 1
                        score += 2;
                        word_match = true;
                        break;
                    }
                }

                if (!word_match)
                {
                    continue;
                }
            }
            else
            {
                // If the requied subject does not require any specific words
                // then increase the score by one
                score++;
            }

            // Check data size
            if (r_sub.data_.size() != n_sub->data_.size())
            {
                //std::cout << "    data size does not match\n";
                continue;
            }

            // Check data
            if (r_sub.data_ != n_sub->data_)
            {
                //std::cout << "    data does not match\n";
                continue;
            }

            //std::cout << "    we got a winner\n";
            subject_match = true;
            node_subjects_c.erase(n_sub);
            break;
        }
        if (subject_match != true)
        {
            return false;
        }
    }
    return score;
}


/* * * * * * * * *
 *  CONNECT TASKS
 * * * * * * * * */

void TaskManager::connectTaskTree(TaskTreeNode& node, std::vector<Subject> parent_subjects, unsigned int depth)
{
    /*
     *
     * TODO: this function is currently quite a mess, a lot of repetitive
     *       stuff going on. Has to be organized
     *
     */

    // If its the root node (depth = 0), then dont look for a task
    if (depth == 0)
    {
        for (auto& child : node.getChildren())
        {
            connectTaskTree(child, parent_subjects, depth + 1);
        }

        return;
    }

    /////////////////////////////    *** PREPARATIONS ***    ////////////////////////////////
    //
    //                  Preparations to manage the upcoming magic

    // A set that contains all tasks that fit the criteria set below. Each matching task has
    // a score that shows how strong the match was.
    std::vector<CandidateTask> candidate_tasks;

    // Variables of the task tree node that are frequently used
    TaskDescriptor& node_task_descriptor = node.getTaskDescriptor();
    const Action& node_action = node_task_descriptor.action_stemmed_;
    TaskInterface& node_interface = node_task_descriptor.getInterfaces()[0];
    std::vector<Subject>& node_subjects = node_interface.input_subjects_;

    // A vector that will contain the incomplete i-subjects of all dependent children
    std::vector<Subject> incomplete_subjects;
    //
    //
    /////////////////////////////////////////////////////////////////////////////////////////

    /*
     * Check if this node depends on the parent node. This is done by
     * checking if the task descriptor contains any incomplete subjects
     */
    if (!node_task_descriptor.getIncompleteSubjects().empty())
    {
        // Debug
        TEMOTO_DEBUG_STREAM ("T'" << node_action
                             << "' depends on parent node, retrieving missing information ...");

        /*
         * Check the node subjects and get the missing information from
         * the parent, making a ALMOST (will get additional information
         * during runtime) complete subject.
         */
        for (auto& n_sub : node_subjects)
        {
            if (n_sub.is_complete_)
            {
                continue;
            }

            // Clean the incomlete subject. This means that all pronouns are removed
            n_sub.words_.clear();

            // go through the parent subjects
            for (auto& p_sub : parent_subjects)
            {
                // If the subject types match, then this is our guy. Copy the contents
                // of the parent subjects into the current incomplete subject
                if (n_sub.type_ == p_sub.type_)
                {
                    /*
                     * TODO: This may create complications at some point, since if parent contains
                     * too much information, e.g., parent has data but the intended child
                     * does not need that, it will get the data regardless. Now if the child
                     * does not have such input interface, its doomed.
                     */
                    n_sub = p_sub;
                }
            }

            // Debug
            TEMOTO_DEBUG_STREAM ("T'" << node_action << "' got: " << n_sub);
        }
    }
    /*
     * Check if some information could already be received from running tasks
     */
    else
    {
        /*
         * Skip all of this jazz when the tasks name is "stop", its a special case
         * and if it is not skipped, then "stop" gets polluted by information just
         * like mentioned above
         */
        if (node_action != "stop")
        {
            for (auto& n_sub : node_subjects)
            {
                // Check each background (asynchronous) task
                for (auto& async_task : asynchronous_tasks_)
                {
                    // Check each output subject of the background task
                    for (auto& t_out_sub : async_task.first->getFirstInterface().output_subjects_)
                    {
                        // Check if types match
                        if (n_sub.type_ != t_out_sub.type_)
                        {
                            continue;
                        }

                        // Check if words are empty
                        if (t_out_sub.words_.empty())
                        {
                            continue;
                        }

                        // Check if the first word matches
                        if (n_sub.words_[0] != t_out_sub.words_[0])
                        {
                            continue;
                        }

                        n_sub = t_out_sub;
                    }
                }
            }
        }
    }

    /*
     * Get the incomplete i-subjects of all dependent children
     */
    for (auto& child : node.getChildren())
    {
        // For every dependent (contains incomplete subjects) child ...
        if (child.getTaskDescriptor().getIncompleteSubjects().size() != 0)
        {
            incomplete_subjects.insert(incomplete_subjects.end(),
                                       child.getTaskDescriptor().getIncompleteSubjects().begin(),
                                       child.getTaskDescriptor().getIncompleteSubjects().end());
        }
    }

    // Debug
    TEMOTO_DEBUG_STREAM ("T'" << node_action << "' has " << incomplete_subjects.size()
                         << " incomplete subjects from children");
    //for (auto& i_s : ){std::cout << i_s;}

    /*
     * If there were any incomplete subjects then ...
     */
    if (!incomplete_subjects.empty())
    {
        /*
         * Elimineate duplicate i-subjects (based only on type)
         */
        for (unsigned int i=incomplete_subjects.size()-1; i<=0; i--)
        {
            Subject& ics = incomplete_subjects[i];

            /*
             * Start looping backwards, starting from the next element to
             * the left of the ics element
             */
            for (unsigned int j=i-1; j<=0; j--)
            {
                if(incomplete_subjects[j].type_ == ics.type_)
                {
                    // Erase the element
                    incomplete_subjects.erase(incomplete_subjects.begin()+i);
                    break;
                }
            }
        }

        // Debug
        TEMOTO_DEBUG_STREAM ("T'" << node_action << "': after eliminating duplicates we have "
                             << incomplete_subjects);
        //for (auto& i_s : incomplete_subjects){std::cout << i_s;}

        /*
         * Clean the incomlete subjects. This means that all pronouns are removed
         */
        for (auto& incomplete_subject : incomplete_subjects)
        {
            incomplete_subject.words_.clear();
        }

        // Debug
        TEMOTO_DEBUG_STREAM ("T'" << node_action << "': looking for a suitable task");

        /*
         * Find task that matches own i-subjects + that has a matching
         * o-subjects with dependent children incomplete i-subjects
         */
        for (auto& task_descriptor : tasks_indexed_)
        {
            // Debug
            TEMOTO_DEBUG_STREAM ("T'" << node_action << "': looking at - " << task_descriptor.getAction());

            // Look for the task with the same action type
            if (task_descriptor.action_stemmed_ != node_action)
            {
                // Check the aliases, if any
                bool alias_found = false;
                for (auto& alias : task_descriptor.aliases_stemmed_)
                {
                    if (alias == node_action)
                    {
                        // Debug
                        TEMOTO_DEBUG_STREAM ("T'" << node_action << "': found action based on it's alias");
                        alias_found = true;
                        break;
                    }
                }

                if (!alias_found)
                {
                    // Debug
                    TEMOTO_DEBUG_STREAM ("T'" << node_action << "': action does not match");
                    continue;
                }
            }

            std::vector<CandidateInterface> candidate_interfaces;

            // Loop over the interfaces and ...
            for (auto& interface : task_descriptor.getInterfaces())
            {
                TEMOTO_DEBUG_STREAM ("T'" << node_action << "': looking at interface no: " << interface.id_);

                // ... look for a word match + STRICT i-subjects match
                unsigned int word_match_score = findWordMatch (interface.input_subjects_, node_subjects);
                if (word_match_score == 0)
                {
                    // Debug
                    TEMOTO_DEBUG_STREAM ("T'" << node_action << "': input subjects do not match. "
                                         << " and we are talking about: \n" << interface.input_subjects_
                                         << " and\n" << node_subjects);
                    continue;
                }

                // ... look for NONSTRICT o-subjects match
                unsigned int output_match_score = findNonstrictMatch (interface.output_subjects_, incomplete_subjects);
                if (output_match_score == 0)
                {
                    // Debug
                    TEMOTO_DEBUG_STREAM ("T'" << node_action
                                         << "': output subjects do not match with dep childern"
                                         << " and we are talking about: \n" << interface.output_subjects_
                                         << " and\n" << incomplete_subjects);
                    continue;
                }

                // And we have a candidate
                candidate_interfaces.emplace_back(interface, word_match_score + output_match_score);
            }

            /*
             * If this task contains any suitable interfaces, pick the one with
             * the best match and push the task to the candidate tasks set
             */
            if (!candidate_interfaces.empty())
            {
                // Debug
                TEMOTO_DEBUG_STREAM ("T'" << node_action << "': IS A SUITABLE CANDIDATE");

                // Sort the candidates with decreasing score and pick the first candidate
                std::sort(candidate_interfaces.begin(),
                          candidate_interfaces.end(),
                          [](CandidateInterface& i1, CandidateInterface& i2)
                          {
                              return i1.score_ > i2.score_;
                          });

                // DEBUG - print out the scores
                //std::cout << "SCORES ";
                //for (auto& inf : candidate_interfaces){std::cout << inf.score_ << ",";} std::cout<<"\n";

                candidate_tasks.emplace_back(task_descriptor, candidate_interfaces[0]);
            }
        }
    }

    /*
     * If there were no incomplete subjects then ...
     */
    else
    {
        /*
         * Find task that matches own i-subjects
         */

        // Debug
        TEMOTO_DEBUG_STREAM ("T'" << node_action << "': looking for a suitable task");

        for (auto& task_descriptor : tasks_indexed_)
        {
            // Debug
            TEMOTO_DEBUG_STREAM ("T'" << node_action << "': looking at - " << task_descriptor.getAction());

            // Look for the task with the same action type
            if (task_descriptor.action_stemmed_ != node_action)
            {
                // Check the aliases, if any
                bool alias_found = false;
                for (auto& alias : task_descriptor.aliases_stemmed_)
                {
                    if (alias == node_action)
                    {
                        // Debug
                        TEMOTO_DEBUG_STREAM ("T'" << node_action << "': found action based on it's alias");
                        alias_found = true;
                        break;
                    }
                }

                if (!alias_found)
                {
                    // Debug
                    TEMOTO_DEBUG_STREAM ("T'" << node_action << "': action does not match");
                    continue;
                }
            }

            std::vector<CandidateInterface> candidate_interfaces;

            // Loop over the interfaces and ...
            for (auto& interface : task_descriptor.getInterfaces())
            {
                // ... look for a word match + STRICT i-subjects match

                unsigned int word_match_score = findWordMatch (interface.input_subjects_, node_subjects);
                if (word_match_score == 0)
                {
                    // Debug
                    TEMOTO_DEBUG_STREAM ("T'" << node_action << "': input subjects do not match. "
                                         << " and we are talking about: " << interface.input_subjects_
                                         << " and " << node_subjects);
                    continue;
                }

                // Debug
                TEMOTO_DEBUG_STREAM ("T'" << node_action << "' strict/word match score: " << word_match_score);

                // And we have a candidate
                candidate_interfaces.emplace_back(interface, word_match_score);
            }

            /*
             * If this task contains any suitable interfaces, pick the one with
             * the best match and push the task to the candidate tasks set
             */
            if (!candidate_interfaces.empty())
            {
                // Sort the candidates with decreasing score and pick the first candidate
                std::sort(candidate_interfaces.begin(),
                          candidate_interfaces.end(),
                          [](CandidateInterface& i1, CandidateInterface& i2)
                          {
                              return i1.score_ > i2.score_;
                          });

                // DEBUG - print out the scores
                //std::cout << "SCORES ";
                //for (auto& inf : candidate_interfaces){std::cout << inf.score_ << ",";} std::cout<<"\n";

                candidate_tasks.emplace_back(task_descriptor, candidate_interfaces[0]);
            }
        }
    }

    /*
     * If no suitable tasks were found then throw an error
     */
    if (candidate_tasks.empty())
    {
        // Throw error
        throw CREATE_ERROR(error::Code::NLP_NO_TASK, "Couldn't find a suitable task for action: " + node_action);
    }

    // Sort the candidates with decreasing order and pick the first candidate
    std::sort(candidate_tasks.begin(),
              candidate_tasks.end(),
              [](CandidateTask& t1, CandidateTask& t2)
              {
                  return t1.ci_.score_ > t2.ci_.score_;
              });

    CandidateTask& cand_task = candidate_tasks[0];

    /*
     * Get a bunch of information out of the prime candidate task
     */
    node_interface.id_ = cand_task.ci_.inf_.id_;
    node_interface.type_ = cand_task.ci_.inf_.type_;
    node_interface.alias_ = node_action; // TODO: This is a hack for Veiko
    node_interface.output_subjects_ = cand_task.ci_.inf_.output_subjects_;
    node_task_descriptor.task_package_name_ = cand_task.td_.task_package_name_;
    node_task_descriptor.aliases_stemmed_ = cand_task.td_.aliases_stemmed_;
    node_task_descriptor.task_lib_path_ = cand_task.td_.task_lib_path_;

    /*
     * Process the child nodes of the tree
     */
    for (auto& child : node.getChildren())
    {
        connectTaskTree(child, node_interface.output_subjects_, depth + 1);
    }
}

/* * * * * * * * *
 *  LOAD AND INITIALIZE TASK TREE
 * * * * * * * * */

void TaskManager::loadAndInitializeTaskTree(TaskTreeNode& node)
{
  // Task descriptor of the node
  TaskDescriptor& node_task_descriptor = node.getTaskDescriptor();

  // If its the root node, then just skip it
  if (node_task_descriptor.getAction() != "ROOT")
  {
    try
    {
      // Load the task
      TEMOTO_DEBUG_STREAM("Loading task '" << node_task_descriptor.getAction() << "'");
      loadTask(node_task_descriptor);

      // Initialize the task
      TEMOTO_DEBUG_STREAM("Instatiating the task '" << node_task_descriptor.getAction() << "'");
      instantiateTask(node);
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
  }

  // Initialize the child tasks
  for (auto& child : node.getChildren())
  {
    loadAndInitializeTaskTree(child);
  }
}


/* * * * * * * * *
 *  LOAD TASK
 * * * * * * * * */

void TaskManager::loadTask(TaskDescriptor& task_descriptor)
{
  /*
   * Create an empty vector for storing the class names (currently a task
   * could have multiple tasks inside it. but it sounds TROUBLESOME
   */
  std::vector<std::string> classes;

  // Get the "path" to its lib file
  std::string path_to_lib = task_descriptor.getLibPath();

  try
  {
    TEMOTO_DEBUG("Loading class from path: %s", path_to_lib.c_str());

    if (class_loaders_.find(path_to_lib) == class_loaders_.end())
    {
      class_loaders_[path_to_lib] = boost::make_shared<class_loader::ClassLoader>(path_to_lib, false);

      // Maintain the name of the synchronous task library, so that it could
      // be unloaded after execution
      if (task_descriptor.getFirstInterface().type_ == "synchronous")
      {
        TEMOTO_DEBUG_STREAM("This is sync task maintaining the library path in a separate set" << std::endl);
        synchronous_task_libs_.push_back(path_to_lib);
      }
    }

    classes = class_loaders_[path_to_lib]->getAvailableClasses<BaseTask>();

    // Done loading
    TEMOTO_DEBUG( "Loaded %lu classes from %s", classes.size(), path_to_lib.c_str() );

    if (classes.empty())
    {
      throw CREATE_ERROR(error::Code::CLASS_LOADER_FAIL, "Could not load the class fom path: " + path_to_lib);
    }

    // Add the name of the class
    task_descriptor.setTaskClassName(classes[0]);
  }

  // Rethrow the exception
  catch(class_loader::ClassLoaderException& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(error::Code::CLASS_LOADER_FAIL, e.what());
  }
}


/* * * * * * * * *
 *  INSTANTIATE TASK
 * * * * * * * * */

void TaskManager::instantiateTask(TaskTreeNode& node)
{
  TaskDescriptor& task_descriptor = node.getTaskDescriptor();

  std::string task_class_name = task_descriptor.getTaskClassName();

  // First check that the task has a "class name"
  if (task_class_name.empty())
  {
    throw CREATE_ERROR(error::Code::NAMELESS_TASK_CLASS, "Task missing a class name.");
  }

//    // Check if there is a class with this name
//    bool task_class_found = false;
//    std::vector<std::string> loaded_classes = class_loader_->getAvailableClasses<BaseTask>();

//    for (std::string loaded_class : loaded_classes)
//    {
//        if (loaded_class == task_class_name)
//        {
//            task_class_found = true;
//            break;
//        }
//    }

//    // If the task was not found, throw an error
//    if (!task_class_found)
//    {
//        throw CREATE_ERROR(error::Code::NO_TASK_CLASS, "Could not find a task class within loaded classes.");
//    }

  // If the task was found, create an instance of it
  try
  {
    // TODO: DO NOT ACCESS PRIVATE MEMBERS DIRECLY ,ie., UNFRIEND THE TASK MANAGER .. or should I?
    TEMOTO_DEBUG( "Instatiating task: %s", task_class_name.c_str());

    node.task_pointer_ = class_loaders_[task_descriptor.getLibPath()]->createInstance<BaseTask>(task_class_name);
    node.task_pointer_->task_package_name_ = task_descriptor.getTaskPackageName();
    node.task_pointer_->task_id_ = id_manager_.generateID();
    node.task_pointer_->class_name_ = task_class_name;
    node.task_pointer_->initializeBase(this);

    // TODO: This is a hack and it should not exist. But currently this is necessary
    // because flow graph internally updates the task descriptors and the updated
    // task descriptors are needed for stopping the tasks correctly
    node.task_descriptor_ptr_ = boost::make_shared<TaskDescriptor>(task_descriptor);

    /*
     * Check if it's a synchronous or an asynchronous task. If it's an asynchronous task, then
     * a copy of its shared pointer is made. This means that synchronous tasks destruct automatically
     * after being executed, since the flow graph owns the last shared pointer and flow graph object
     * is always destructed after execution (see TaskManager::executeSFT).
     */
    if (task_descriptor.getFirstInterface().type_ == "asynchronous")
    {
      TEMOTO_DEBUG_STREAM("This is an Asynchronous task, making a copy of the shared ptr\n");
      asynchronous_tasks_.push_back( {node.task_descriptor_ptr_, node.task_pointer_} );
    }
    else
    {
      TEMOTO_DEBUG_STREAM("This is a Synchronous task, making a copy of the shared ptr\n");
      synchronous_tasks_.push_back( {node.task_descriptor_ptr_, node.task_pointer_} );
    }

    return;
  }
  catch(class_loader::ClassLoaderException& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(error::Code::CLASS_LOADER_FAIL, e.what());
  }
}

/* * * * * * * * *
 *  UNLOAD TASK LIB
 * * * * * * * * */

void TaskManager::unloadTaskLib(std::string path_to_lib)
{
  try
  {
    TEMOTO_INFO_STREAM("Unloading library: " << path_to_lib );
    //delete class_loaders_[path_to_lib];
    class_loaders_.erase(path_to_lib);
  }
  catch(class_loader::ClassLoaderException& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(error::Code::CLASS_LOADER_FAIL, e.what());
  }
}


/* * * * * * * * *
 *  MAKE FLOW GRAPH
 * * * * * * * * */

void TaskManager::makeFlowGraph(TaskTreeNode& node, tbb::flow::graph& flow_graph)
{
    TaskDescriptor node_task_descriptor = node.getTaskDescriptor();
    boost::shared_ptr<TaskDescriptor> node_task_descriptor_ptr = node.task_descriptor_ptr_;

    // If its the root node, then create the start node
    if (node_task_descriptor.getAction() == "ROOT")
    {
        node.root_fgn_ = std::make_unique<tbb::flow::broadcast_node<Subjects>>(flow_graph);
    }

    // Create a continue node
    else
    {
        node.task_fgn_ = std::make_unique< tbb::flow::function_node<Subjects, Subjects> >
                (flow_graph
                 , tbb::flow::serial
                 , TaskContainer(node.task_pointer_, node_task_descriptor_ptr));
    }

    // Do the same with child nodes
    for (auto& child : node.getChildren())
    {
        makeFlowGraph(child, flow_graph);
    }
}


/* * * * * * * * *
 *  CONNECT FLOW GRAPH
 * * * * * * * * */

void TaskManager::connectFlowGraph(TaskTreeNode& node)
{
  // If its the root node, then create the start edge
  if (node.getTaskDescriptor().getAction() == "ROOT")
  {
    // Connect parent flow graph node with child flow graph nodes
    for (auto& child : node.getChildren())
    {
      tbb::flow::make_edge(*node.root_fgn_, *child.task_fgn_);
      connectFlowGraph(child);
    }
  }
  else
  {
    // Connect parent flow graph node with child flow graph nodes
    for (auto& child : node.getChildren())
    {
      tbb::flow::make_edge(*node.task_fgn_, *child.task_fgn_);
      connectFlowGraph(child);
    }
  }
}


/* * * * * * * * *
 *  STOP TASK SERVICE CALLBACK
 * * * * * * * * */

bool TaskManager::stopTaskCallback( temoto_2::StopTask::Request& req,
                                    temoto_2::StopTask::Response& res)
{
  // Debug
  TEMOTO_DEBUG( "Received a request to stop task. Action = '%s'; what = '%s'"
             , req.action.c_str(), req.what.c_str());

  try
  {
    //std::cout << "D0\n";
    stopTask(req.action, req.what);

    //std::cout << "D1\n";
    res.code = 0;
    res.message = "task stopped";
  }

  catch(error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }
  catch(...)
  {
    CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
  }

  return true;
}


/* * * * * * * * *
 *  STOP TASK
 * * * * * * * * */

void TaskManager::stopTask(std::string action, std::string what)
{
  bool task_stopped = false;

  // Debug
  TEMOTO_DEBUG_STREAM ("No of asynchronous tasks: " << asynchronous_tasks_.size());

  /*
   * Stop by action and "what"
   */
  if( !action.empty() && !what.empty() )
  {
    // Debug
    TEMOTO_DEBUG_STREAM ("Stopping the task based on 'action' and 'what'");

    // Look for the task
    for (auto task_it = asynchronous_tasks_.begin();
         task_it != asynchronous_tasks_.end();
         ++task_it)
    {
      // Look for the action
      bool action_found = false;
      for (auto& alias : task_it->first->aliases_stemmed_)
      {
        if (alias == action)
        {
          action_found = true;
          break;
        }
      }

      // If the action was not found then continue
      if (!action_found)
      {
        continue;
      }

      // Look for the what
      Subjects subjects = task_it->first->getFirstInputSubjects(); // copy
      try
      {
        Subject subject = getSubjectByType("what", subjects);

        if (subject.words_.empty() || subject.words_[0] != what)
        {
          continue;
        }
      }
      catch(std::string& e)
      {
        throw CREATE_ERROR(error::Code::SUBJECT_NOT_FOUND, e);
      }


      // Debug
      TEMOTO_DEBUG_STREAM ("Found the task, stopping it");

      // Remove the task
      (task_it->second)->stopTask();
      asynchronous_tasks_.erase(task_it);
      task_stopped = true;
      break;
    }
  }

  /*
   * Stop by action
   */
  else if( !action.empty() )
  {
    // Debug
    TEMOTO_DEBUG_STREAM ("Stopping the task based on 'action'");

    // Look for the task
    for (auto task_it = asynchronous_tasks_.begin();
         task_it != asynchronous_tasks_.end();
         ++task_it)
    {
      // Look for the action
      bool action_found = false;
      for (auto& alias : task_it->first->aliases_stemmed_)
      {
        if (alias == action)
        {
          action_found = true;
          break;
        }
      }

      // If the action was not found then continue
      if (!action_found)
      {
        continue;
      }

      // Debug
      TEMOTO_DEBUG_STREAM ("Found the task, stopping it");

      // Remove the task
      (task_it->second)->stopTask();
      asynchronous_tasks_.erase(task_it);
      task_stopped = true;
      break;
    }
  }

  /*
   * Stop by "what"
   * In this case everything that corresponds to what is closed
   * ex: "start THE CAMERA and show IT (the camera) in rviz" - both tasks are closed
   */
  else if( !what.empty() )
  {
    // Debug
    TEMOTO_DEBUG_STREAM ("Stopping the task based on 'what'");

//    for (auto task_it = asynchronous_tasks_.begin();
//         task_it != asynchronous_tasks_.end();
//         /* empty */)
//    {

//      // Look for the what
//      TEMOTO_ERROR_STREAM("task name = " << task_it->first->getAction());
//      Subjects subjects = task_it->first->getFirstOutputSubjects(); // copy
//      TEMOTO_ERROR_STREAM(subjects);

//      if (task_it->first->getAction() == "accept")
//      {
//        // task_it->first->task_interfaces_.clear();
//        std::string libpath = task_it->first->getLibPath();
//        asynchronous_tasks_.erase(task_it);
//        class_loaders_.erase(libpath);
//        //class_loaders_[libpath]->unloadLibrary();
//        //delete class_loaders_[libpath];
//      }
//      else
//      {
//        task_it++;
//      }
//    }

//    TEMOTO_DEBUG_STREAM ("Second round");

//    for (auto task_it = asynchronous_tasks_.begin();
//         task_it != asynchronous_tasks_.end();
//         task_it++)
//    {

//      // Look for the what

//      if (task_it->first->getAction() == "provide")
//      {
//        std::vector<Data>& v_data = task_it->first->getFirstOutputSubjects()[0].data_;
//        std::cout << "D0\n";
//        // Data dat = v_data[0];
//        //dat.value = boost::any(v_data[0].value);
//        //v_data[0] = dat;
//        boost::any val = boost::any_cast<std::string>(std::string("Sum data jea"));
//        v_data[0].value = val;
//      }
//      TEMOTO_ERROR_STREAM("task name = " << task_it->first->getAction());
//      Subjects in_subjects = task_it->first->getFirstInputSubjects(); // copy
//      Subjects out_subjects = task_it->first->getFirstOutputSubjects(); // copy
//      TEMOTO_ERROR_STREAM(in_subjects);
//      TEMOTO_ERROR_STREAM(out_subjects);
//    }

    // Look for the task
    for (auto task_it = asynchronous_tasks_.begin();
         task_it != asynchronous_tasks_.end();
         /* empty */)
    {

      // Look for the what
//      TEMOTO_INFO_STREAM("task name = " << task_it->first->getAction());

      //Subjects subjects2 = task_it->first->getFirstOutputSubjects(); // copy
      Subjects subjects = task_it->first->getFirstInputSubjects(); // copy

      //TEMOTO_INFO_STREAM(subjects);

      //std::cout << "D0_1\n";
      try
      {
        Subject subject = getSubjectByType("what", subjects);
//        TEMOTO_ERROR_STREAM("is: '" << subject.words_[0] << "' should be: '" << what << "' ");
        if (subject.words_.empty() || subject.words_[0] != what)
        {
          task_it++;
          continue;
        }
      }
      catch(std::string& e)
      {
        throw CREATE_ERROR(error::Code::SUBJECT_NOT_FOUND, e);
//        TEMOTO_ERROR_STREAM("this is bs but lets continue");
//        task_it++;
//        continue;
      }

      //std::cout << "D0_2\n";

      std::string lib_path = task_it->first->getLibPath();

      // Debug
      TEMOTO_DEBUG_STREAM ("Found task '" << lib_path << "'. Stopping it");

      // Remove the task, task_it is pointed to the next element
      (task_it->second)->stopTask();
      asynchronous_tasks_.erase(task_it);

      //std::cout << "D0_3\n";

      // TODO: the library should not be unloaded here. It should be done in the unloadTasks method
      // TODO: And even if its unloded here, issue #3 happens (refer to github)
//      unloadTaskLib(lib_path);

      //std::cout << "D0_4\n";
      task_stopped = true;
    }
  }

  if (!task_stopped)
  {
    // If nothing was specified, then throw error
    throw CREATE_ERROR(error::Code::UNSPECIFIED_TASK, "Task 'action' and 'what' unspecified.");
  }
}


/* * * * * * * * *
 *  INDEX TASKS SERVICE CALLBACK
 * * * * * * * * */

void TaskManager::indexTasksCallback(temoto_2::IndexTasks index_msg)
{
    TEMOTO_DEBUG( "Received a request to index tasks at '%s'", index_msg.directory.c_str());
    try
    {
        // Wait until the action execution engine is not busy
        while (action_executioner_busy_)
        {
            ros::Duration(0.05).sleep();
            TEMOTO_DEBUG_STREAM("Waiting for action executioner ...");
        }
        TEMOTO_DEBUG_STREAM("Done waiting. Starting indexing process");

        boost::filesystem::directory_entry dir(index_msg.directory);
        indexTasks(dir, 1);
        TEMOTO_DEBUG_STREAM("Browsed and indexed the tasks successfully");
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
}


/* * * * * * * * *
 *  THREAD JOINING TIMER CALLBACK
 * * * * * * * * */

void TaskManager::threadJoiningTimerCallback(const ros::TimerEvent& e)
{
  //TEMOTO_INFO_STREAM(flow_graph_futures_.size() << " threads are currently running");

  // Iterate through the thread vector and check if they are joinable
  for(auto future_it = flow_graph_futures_.begin();
      future_it != flow_graph_futures_.end();
      /* empty */)
  {
    if(future_it->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      TEMOTO_INFO_STREAM("An action thread has finished");
      flow_graph_futures_.erase(future_it);
    }
    else
    {
      future_it++;
    }
  }

  // Check if a synchronous task has finished
  for (auto itr = synchronous_tasks_.begin();
       itr != synchronous_tasks_.end();
       /* empty */)
  {
    TEMOTO_INFO_STREAM("Use count of this synchronous task is: " << itr->second.use_count());
    if (itr->second.use_count() == 1)
    {
      TEMOTO_INFO_STREAM("Destructing synchronous task");
      synchronous_tasks_.erase(itr);
    }
    else
    {
      itr++;
    }
  }
}

TaskManager::~TaskManager()
{
  TEMOTO_INFO("Stopping all actions");

  // Stop the synchronous actions
  for (auto itr = synchronous_tasks_.begin();
       itr != synchronous_tasks_.end();
       /* empty */)
  {
    TEMOTO_INFO_STREAM("Use count of this synchronous task is: " << itr->second.use_count());
    if (itr->second.use_count() == 1)
    {
      TEMOTO_INFO_STREAM("Destructing synchronous task");
      synchronous_tasks_.erase(itr);
    }
    else if (itr->second.use_count() > 1)
    {
      // Tell the task to finish its business
      itr->second->stopTask();

      // Wait until the task has actually finished
      // TODO: also add a timeout for the while loop. if the timeout is reached
      // then somehow force the action to stop
      while (!itr->second->taskFinished() || itr->second.use_count() > 1)
      {
        TEMOTO_INFO_STREAM("Waiting the Synchronous action to finish ...");
        ros::Duration(0.1).sleep();
      }

      // Erase the task
      synchronous_tasks_.erase(itr);
    }
  }

  // Stop the asynchronous actions
  for (auto itr = asynchronous_tasks_.begin();
       itr != asynchronous_tasks_.end();
       /* empty */)
  {
    TEMOTO_INFO_STREAM("Use count of this synchronous task is: " << itr->second.use_count());
    if (itr->second.use_count() == 1)
    {
      TEMOTO_INFO_STREAM("Destructing synchronous task");
      asynchronous_tasks_.erase(itr);
    }
    else if (itr->second.use_count() > 1)
    {
      // Tell the task to finish its business
      itr->second->stopTask();

      // Wait until the task has actually finished
      // TODO: also add a timeout for the while loop. if the timeout is reached
      // then somehow force the action to stop
      while (!itr->second->taskFinished() || itr->second.use_count() > 1)
      {
        TEMOTO_INFO_STREAM("Waiting the Asynchronous action to finish ...");
        ros::Duration(0.1).sleep();
      }

      // Erase the task
      asynchronous_tasks_.erase(itr);
    }
  }

  TEMOTO_INFO("All actions stopped");
}

}// END of TTP namespace
