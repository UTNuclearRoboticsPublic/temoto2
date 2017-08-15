/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: * CATCH ALL EXEPTIONS AND RETHROW AS
 *                          TEMOTO ERROR !!!
 *                        * Start using ros cpp naming conventions
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "core/task_handler/task_handler.h"

/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskHandler::TaskHandler (std::string system_prefix)
    : system_prefix_(system_prefix)
{
    // Initialize the vector of indexed tasks.
    tasks_indexed_ = new std::vector <TaskInfo>(1);

    // Start the servers
    class_loader_ = new class_loader::MultiLibraryClassLoader(false);
    this->stop_task_server_ = n_.advertiseService (system_prefix_ + "/stop_task", &TaskHandler::stopTaskCallback, this);
    this->index_tasks_server_ = n_.advertiseService (system_prefix_ + "/index_tasks", &TaskHandler::indexTasksCallback, this);
}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string task_to_find, const std::vector <TaskInfo>& tasks)
{
    std::vector <TaskInfo> tasksFound;
    for (auto& task : tasks)
    {
        if (task_to_find.compare(task.getName()) == 0)
        {
            tasksFound.push_back(task);
        }
    }
    return tasksFound;
}


/* * * * * * * * *
 *  FIND TASK LOCAL
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTaskLocal(std::string task_to_find)
{
    return findTask(task_to_find, *(this->tasks_indexed_));
}


/* * * * * * * * *
 *  FIND TASK RUNNING
 * * * * * * * * */
/*
std::vector <TaskInfo> TaskHandler::findTaskRunning(std::string task_to_find)
{
    return findTask(task_to_find, this->running_tasks_);
}
*/

/* * * * * * * * *
 *  FIND TASK FROM FILESYSTEM
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTaskFilesys(std::string task_to_find, boost::filesystem::directory_entry base_path, int search_depth)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "findTaskFilesys";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_);

    boost::filesystem::path current_dir (base_path);
    boost::filesystem::directory_iterator end_itr;
    std::vector <TaskInfo> tasksFound;

    try
    {
        // Start looking the files inside current directory
        for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
        {

            // if its a directory and depth limit is not there yet, go inside it
            if ( boost::filesystem::is_directory(*itr) & (search_depth > 0) )
            {
                std::vector<TaskInfo> subTasksFound = findTaskFilesys( task_to_find, *itr, (search_depth - 1));

                // Append the subtasks if not empty
                if ( !subTasksFound.empty() )
                {
                    tasksFound.insert(std::end(tasksFound), std::begin(subTasksFound), std::end(subTasksFound));
                }
            }

            // if its a file and matches the desc file name, process the file
            else if ( boost::filesystem::is_regular_file(*itr) &
                      ((*itr).path().filename().compare(descriptionFile) == 0) )
            {
                //int res = processDesc (taskType, (*itr).path().string());
                try
                {
                    // Create a description processor object
                    // I THINK THIS SHOULD NOT BE CREATED EVERY SINGLE TIME
                    boost::filesystem::path hackdir ((*itr)); //HACKATON
                    DescriptionProcessor descProcessor( hackdir.parent_path().string() );

                    // Get TaskInfo
                    TaskInfo taskInfo = descProcessor.getTaskInfo();
                    tasksFound.push_back( taskInfo );
                }

                catch( error::ErrorStack & e )
                {
                    // Append the error to local ErrorStack
                    e.emplace_back( coreErr::FORWARDING,
                                    error::Subsystem::CORE,
                                    e.back().getUrgency(),
                                    formatMessage("", this->class_name_, method_name_) + "FORWARDING");

                    this->error_handler_.append(e);
                }
            }
        }
        return tasksFound;
    }

    catch (std::exception& e)
    {
        ROS_ERROR("%s %s", prefix.c_str(), e.what());
        return tasksFound;
    }

    catch(...)
    {
        ROS_ERROR("%s Unhandled exception", prefix.c_str());
        return tasksFound;
    }
}

/* * * * * * * * *
 *  INDEX TASKS
 * * * * * * * * */

void TaskHandler::indexTasks (boost::filesystem::directory_entry base_path, int search_depth)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "indexTasks";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_).c_str();

    try
    {
        *tasks_indexed_ = findTaskFilesys ("", base_path, search_depth);
    }
    catch (...)
    {
        ROS_ERROR("%s Unhandled exception", prefix.c_str());
    }
}


/* * * * * * * * *
 *  GET THE INDEX TASKS
 * * * * * * * * */

std::vector <TaskInfo>* TaskHandler::getIndexedTasks()
{
    return this->tasks_indexed_;
}


/* * * * * * * * *
 *  LOAD TASK
 * * * * * * * * */

bool TaskHandler::loadTask(RunningTask& task)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "loadTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_).c_str();

    // Create an empty vector for storing the class names (currently a task
    // could have multiple tasks inside it. but it sounds TROUBLESOME
    std::vector<std::string> classes;

    // Get the "path" to its lib file
    std::string path_to_lib =  task.task_info_.getLibPath();

    try
    {
        // Start loading a task library
        class_loader_->loadLibrary(path_to_lib);
        classes = class_loader_->getAvailableClassesForLibrary<Task>(path_to_lib);

        // Done loading
        ROS_DEBUG( "%s Loaded %lu classes from %s", prefix.c_str(), classes.size(), path_to_lib.c_str() );

        // Add the name of the class
        task.task_info_.class_name_ = classes[0];

        return true;
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("%s ClassLoaderException: %s", prefix.c_str(), e.what());
        return false;
    }

    return false;
}


/* * * * * * * * *
 *  INSTANTIATE TASK
 * * * * * * * * */

bool TaskHandler::instantiateTask(RunningTask& task)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "instansiateTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_);

    std::string taskClassName = task.task_info_.getClassName();

    // First check that the task has a "class name"
    if (taskClassName.empty())
    {
        ROS_ERROR( "%s Unset 'class name'", prefix.c_str() );
        return false;
    }

    // Check if there is a class with this name
    bool task_class_found = false;
    std::vector<std::string> loadedClasses = class_loader_->getAvailableClasses<Task>();

    for (std::string singleClass : loadedClasses)
    {
        if (singleClass.compare(taskClassName) == 0)
        {
            task_class_found = true;
            break;
        }
    }

    // If the task was found, create an instance of it
    if (task_class_found)
    {
        try
        {
            ROS_DEBUG( "%s instatiating task: %s", prefix.c_str(), taskClassName.c_str());
            task.task_pointer_ = class_loader_->createInstance<Task>(taskClassName);

            return true;
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("%s ClassLoaderException: %s", prefix.c_str(), e.what());
            return false;
        }
    }
    else
    {
        ROS_DEBUG( "%s task: '%s' was not found", prefix.c_str(), taskClassName.c_str());
        return false;
    }

    return true;
}


/* * * * * * * * *
 *  START TASK WITHOUT ARGUMENTS
 *
 * TODO: CHECK IF THE TASK EVEN EXISTS IN THE LIST OF RUNNING TASKS
 * * * * * * * * */

bool TaskHandler::startTask(RunningTask& task)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "startTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_);

    try
    {
        ROS_DEBUG( "%s starting task: %s", prefix.c_str(), task.task_info_.getName().c_str());
        task.task_pointer_->startTask();
        return true;
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("%s ClassLoaderException: %s", prefix.c_str(), e.what());
        return false;
    }

    return false;
}

/* * * * * * * * *
 *  START TASK WITH ARGUMENTS
 *
 *  TODO: * check the running tasks lists before doing anything
 * * * * * * * * */

bool TaskHandler::startTask(RunningTask& task, std::vector<boost::any> arguments)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "startTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_).c_str();

    // If the list is empty then ...
    if (arguments.empty())
    {
        startTask(task);
        return true;
    }
    else
    {
        try
        {
            ROS_DEBUG( "%s starting task (with arguments): %s", prefix.c_str(), task.task_info_.getName().c_str());
            task.task_pointer_->startTask(0, arguments);
            return true;
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("%s ClassLoaderException: %s", prefix.c_str(), e.what());
            return false;
        }
    }

    return false;
}


/* * * * * * * * *
 *  START TASK THREAD
 * * * * * * * * */

bool TaskHandler::startTaskThread(RunningTask& task, std::vector<boost::any> arguments)
{
    using memfunc_type = bool (TaskHandler::*)(RunningTask&, std::vector<boost::any>);
    memfunc_type memfunc = &TaskHandler::startTask;

    task.task_thread_ = std::thread(memfunc, this, std::ref(task), arguments);
    return true;
}


/* * * * * * * * *
 *  EXECUTE TASK
 * * * * * * * * */

bool TaskHandler::executeTask(TaskInfo task_info, std::vector<boost::any> arguments)
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "executeTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_).c_str();

    // Create a RunningTask object
    RunningTask task_to_run;
    task_to_run.task_info_ = task_info;

    /*
     * Use the name of the task to load in the task class. task handler returns the internal
     * specific name of the task which is used in the next step.
     */
    ROS_DEBUG("%s Executing task '%s' ...", prefix.c_str(), task_to_run.task_info_.getName().c_str());

    if ( !loadTask(task_to_run) )
    {
        // throw error
        return false;
    }

    /*
     * Create an instance of the task based on the class name. a task .so file might
     * contain multiple classes, therefore path is not enough and specific name
     * must be used
     */
    if ( instantiateTask(task_to_run) )
    {
        /*
         * MOVE the task info into the list of running tasks. If the move operation is done
         * after starting the task, the process will likely crash.
         */
        ROS_DEBUG("%s Moving the task into the 'running_tasks' vector ...", prefix.c_str());
        running_tasks_.push_back (std::move(task_to_run));

        //Start the task
        if ( !startTaskThread(running_tasks_.back(), arguments) )
        {
            stopTask(running_tasks_.back().task_info_.getName());
            return false;
        }

    }

    return true;
}


/* * * * * * * * *
 *  STOP TASK
 *
 *  TODO: when multiple tasks with the same name are found then do something more reasonable than
 *        just stopping the first one.
 * * * * * * * * */

bool TaskHandler::stopTask(std::string task_name)
{
    // Name of the method
    const std::string method_name_ = "stopTask";
    std::string prefix = formatMessage(this->system_prefix_, this->class_name_, method_name_);

    // Find the task by iterating through the list of running tasks
    for (auto it = this->running_tasks_.begin(); it != this->running_tasks_.end(); it++)
    {
        // If a task was found then delete it
        if ((*it).task_info_.getName().compare(task_name) == 0)
        {
            try
            {
                ROS_DEBUG( "%s Stopping and erasing task: %s", prefix.c_str(), (*it).task_info_.getName().c_str());
                /*
                 * First, call the "stopTask", given that a task is checking the internal "stop_task_" flag
                 * TODO: Implement a way for brutally forcing  the task thread to stop in case the task is
                 * stuck inside a loop of some sort
                 */
                (*it).task_pointer_->stopTask();

                // If the task is stopped, join the thread
                (*it).task_thread_.join();

                // Delete the task entry, which also calls the destructor of the task
                running_tasks_.erase (it);

                ROS_DEBUG( "%s Task stopped", prefix.c_str());

                return true;
            }

            catch(class_loader::ClassLoaderException& e)
            {
                ROS_ERROR("%s ClassLoaderException: %s", prefix.c_str(), e.what());
                return false;
            }

            catch(...)
            {
                ROS_ERROR("%s exception happened", prefix.c_str());
                return false;
            }
        }
    }

    ROS_ERROR("%s exception happened", prefix.c_str());
    return false;
}


/* * * * * * * * *
 *  UNLOAD LIB
 * * * * * * * * */

bool TaskHandler::unloadTaskLib(std::string path_to_lib)
{
    ROS_DEBUG( "[TaskHandler/unloadTaskLib] unloading library");
    class_loader_->unloadLibrary(path_to_lib);
    return true;
}


/* * * * * * * * *
 *  START TASK SERVICE CALLBACK
 * * * * * * * * */

/*
bool TaskHandler::startTaskCallback (temoto_2::startTask::Request& req,
                                     temoto_2::startTask::Response& res)
{
    ROS_DEBUG( "[%s/TaskHandler::startTaskCallback] Received a request to start task '%s'", this->system_prefix_.c_str(), req.task_name.c_str());

    // Load the task
    ROS_DEBUG( "[%s/TaskHandler::startTaskCallback] Loading the task", this->system_prefix_.c_str());
    std::string taskClassName = loadTask(req.task_name);

    // Check if the loading proccess was successful
    if ( taskClassName.empty() )
    {
        res.code = 1;
        res.message = "Unable to load the task";
        return true;
    }

    // Create an instance of the task based on the class name. a task .so file might
    // contain multiple classes, therefore path is not enough and specific name
    // must be used
    ROS_DEBUG( "[%s/TaskHandler::startTaskCallback] Creating an instance of the task", this->system_prefix_.c_str());

    if ( !instantiateTask(taskClassName) )
    {
        res.code = 1;
        res.message = "Unable to create an instance of the task";
        return true;
    }

    //Start the task
    ROS_DEBUG( "[%s/TaskHandler::startTaskCallback] Starting the task", this->system_prefix_.c_str());

    if ( startTask(taskClassName, task.second) )
    {
        taskHandler.stopTask(taskClassName);
    }
}
*/

/* * * * * * * * *
 *  STOP TASK SERVICE CALLBACK
 * * * * * * * * */

bool TaskHandler::stopTaskCallback (temoto_2::stopTask::Request& req,
                                    temoto_2::stopTask::Response& res)
{
    ROS_DEBUG( "[%s/TaskHandler::stopTaskCallback] Received a request to stop task '%s'", this->system_prefix_.c_str(), req.name.c_str());
    if ( stopTask(req.name) )
    {
        res.code = 0;
        res.message = "task stopped";
    }

    else
    {
        res.code = 1;
        res.message = "failed to stop the task";
    }

    return true;
}


/* * * * * * * * *
 *  INDEX TASKS SERVICE CALLBACK
 * * * * * * * * */

bool TaskHandler::indexTasksCallback (temoto_2::indexTasks::Request& req,
                         temoto_2::indexTasks::Response& res)
{
    ROS_DEBUG( "[%s/TaskHandler::indexTasksCallback] Received a request to index tasks at '%s'", this->system_prefix_.c_str(), req.directory.c_str());
    try
    {
        boost::filesystem::directory_entry dir(req.directory);
        indexTasks(dir, 1);
        res.code = 0;
        res.message = "Browsed and indexed the tasks successfully";

        return true;
    }
    catch(...)
    {
        ROS_ERROR("[%s/TaskHandler::indexTasksCallback] Failed to index the tasks", this->system_prefix_.c_str());
        res.code = 1;
        res.message = "Failed to index the tasks";
        return false;
    }
}
