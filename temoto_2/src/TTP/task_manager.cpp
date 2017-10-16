/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: * CATCH ALL EXEPTIONS AND RETHROW AS
 *                          TEMOTO ERROR !!!
 *                        * Start using ros cpp naming conventions
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "TTP/task_manager.h"

/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskManager::TaskManager (std::string system_prefix)
    : system_prefix_(system_prefix)
{
    // Initialize the vector of indexed tasks
    tasks_indexed_ = new std::vector <TaskDescriptor>(1);

    // Construct the classloader
    class_loader_ = new class_loader::MultiLibraryClassLoader(false);

    // Start the servers
    stop_task_server_ = n_.advertiseService (system_prefix_ + "/stop_task", &TaskManager::stopTaskCallback, this);
    index_tasks_server_ = n_.advertiseService (system_prefix_ + "/index_tasks", &TaskManager::indexTasksCallback, this);

    // Start subscribers
    stop_task_subscriber_ = n_.subscribe("temoto/stop_task", 10, &TaskManager::stopTaskMsgCallback, this);
}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskDescriptor> TaskManager::findTask(std::string task_to_find, const std::vector <TaskDescriptor>& tasks)
{
    std::vector <TaskDescriptor> tasks_found;
    for (auto& task : tasks)
    {
        if (task_to_find.compare(task.getName()) == 0)
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
    return findTask(task_to_find, *(this->tasks_indexed_));
}


/* * * * * * * * *
 *  FIND TASK RUNNING
 * * * * * * * * */
/*
std::vector <TaskDescriptor> TaskManager::findTaskRunning(std::string task_to_find)
{
    return findTask(task_to_find, this->running_tasks_);
}
*/

/* * * * * * * * *
 *  FIND TASK FROM FILESYSTEM
 * * * * * * * * */

std::vector <TaskDescriptor> TaskManager::findTaskFilesys( std::string task_to_find,
                                                     boost::filesystem::directory_entry base_path,
                                                     int search_depth)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

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
                //int res = processDesc (taskType, (*itr).path().string());
                try
                {
                    // Create a description processor object
                    // I THINK THIS SHOULD NOT BE CREATED EVERY SINGLE TIME
                    boost::filesystem::path hackdir ((*itr)); //HACKATON
                    DescriptionProcessor desc_processor( hackdir.parent_path().string() );

                    // Get TaskDescriptor
                    TaskDescriptor task_info = desc_processor.getTaskDescriptor();
                    tasks_found.push_back( task_info );
                }

                catch( error::ErrorStackUtil & e )
                {
                    // Append the error to local ErrorStack
                    e.forward( prefix );
                    error_handler_.append(e);
                }
            }
        }
        return tasks_found;
    }
    catch (std::exception& e)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::FIND_TASK_FAIL,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + e.what(),
                                     ros::Time::now() );
    }

    catch(...)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::UNHANDLED,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Received an unhandled exception",
                                     ros::Time::now() );
    }
}

/* * * * * * * * *
 *  INDEX TASKS
 * * * * * * * * */

void TaskManager::indexTasks (boost::filesystem::directory_entry base_path, int search_depth)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    try
    {
        *tasks_indexed_ = findTaskFilesys ("", base_path, search_depth);
    }
    catch( error::ErrorStackUtil & e )
    {
        // Rethrow the exception
        e.forward( prefix );
        throw e;
    }
}


/* * * * * * * * *
 *  GET THE INDEX TASKS
 * * * * * * * * */

std::vector <TaskDescriptor>* TaskManager::getIndexedTasks()
{
    return this->tasks_indexed_;
}


/* * * * * * * * *
 *  INDEX TASKS SERVICE CALLBACK
 * * * * * * * * */

bool TaskManager::indexTasksCallback( temoto_2::indexTasks::Request& req,
                                      temoto_2::indexTasks::Response& res)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    ROS_DEBUG( "%s Received a request to index tasks at '%s'", prefix.c_str(), req.directory.c_str());
    try
    {
        boost::filesystem::directory_entry dir(req.directory);
        indexTasks(dir, 1);
        res.code = 0;
        res.message = "Browsed and indexed the tasks successfully";

        return true;
    }
    catch( error::ErrorStackUtil & e )
    {
        // Append the error to local ErrorStack
        e.forward( prefix );
        error_handler_.append(e);

        res.code = 1;
        res.message = "Failed to index the tasks";
        return true;
    }
}


/* * * * * * * * *
 *  LOAD TASK
 * * * * * * * * */

void TaskManager::loadTask(RunningTask& task)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    /*
     * Create an empty vector for storing the class names (currently a task
     * could have multiple tasks inside it. but it sounds TROUBLESOME
     */
    std::vector<std::string> classes;

    // Get the "path" to its lib file
    std::string path_to_lib =  task.task_info_.getLibPath();

    try
    {
        // Start loading a task library
        ROS_INFO("Loading class from path: %s", path_to_lib.c_str());
        class_loader_->loadLibrary(path_to_lib);
        classes = class_loader_->getAvailableClassesForLibrary<Task>(path_to_lib);

        // Done loading
        ROS_DEBUG( "%s Loaded %lu classes from %s", prefix.c_str(), classes.size(), path_to_lib.c_str() );

        // Add the name of the class
        task.task_info_.class_name_ = classes[0];
    }

    // Rethrow the exception
    catch(class_loader::ClassLoaderException& e)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::CLASS_LOADER_FAIL,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + e.what(),
                                     ros::Time::now() );
    }
}


/* * * * * * * * *
 *  UNLOAD LIB
 * * * * * * * * */

void TaskManager::unloadTaskLib(std::string path_to_lib)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    try
    {
        ROS_DEBUG( "[TaskManager/unloadTaskLib] unloading library");
        class_loader_->unloadLibrary(path_to_lib);
    }
    catch(class_loader::ClassLoaderException& e)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::CLASS_LOADER_FAIL,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + e.what(),
                                     ros::Time::now() );
    }
}

/* * * * * * * * *
 *  INSTANTIATE TASK
 * * * * * * * * */

void TaskManager::instantiateTask(RunningTask& task)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    std::string task_class_name = task.task_info_.getClassName();

    // First check that the task has a "class name"
    if (task_class_name.empty())
    {
        throw error::ErrorStackUtil( coreErr::NAMELESS_TASK_CLASS,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Task missing a class name",
                                     ros::Time::now() );
    }

    // Check if there is a class with this name
    bool task_class_found = false;
    std::vector<std::string> loaded_classes = class_loader_->getAvailableClasses<Task>();

    for (std::string loaded_class : loaded_classes)
    {
        if (loaded_class == task_class_name)
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
            ROS_DEBUG( "%s instatiating task: %s", prefix.c_str(), task_class_name.c_str());
            task.task_pointer_ = class_loader_->createInstance<Task>(task_class_name);

            return;
        }
        catch(class_loader::ClassLoaderException& e)
        {
            // Rethrow the exception
            throw error::ErrorStackUtil( coreErr::CLASS_LOADER_FAIL,
                                         error::Subsystem::CORE,
                                         error::Urgency::HIGH,
                                         prefix + e.what(),
                                         ros::Time::now() );
        }
    }
    else
    {
        throw error::ErrorStackUtil( coreErr::NO_TASK_CLASS,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Could not find a task class within loaded classes",
                                     ros::Time::now() );
    }
}


/* * * * * * * * *
 *  START TASK
 *
 *  TODO: * check the running tasks lists before doing anything
 * * * * * * * * */

void TaskManager::startTask(RunningTask& task, std::vector<boost::any> arguments)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    try
    {
        ROS_DEBUG( "%s starting task: %s", prefix.c_str(), task.task_info_.getName().c_str());
        task.task_pointer_->setID( id_manager_.generateID() );
        task.task_pointer_->startTaskAutostop( 0, arguments );
    }

    catch(...)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::UNHANDLED,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Received an unhandled exception",
                                     ros::Time::now() );
    }
}

/* * * * * * * * *
 *  EXECUTE TASK
 * * * * * * * * */

bool TaskManager::executeTask(TaskDescriptor task_info, std::vector<boost::any> arguments)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    bool start_attempted = false;

    try
    {
        // Create a RunningTask object
        RunningTask task_to_run;
        task_to_run.task_info_ = task_info;

        /*
         * Use the name of the task to load in the task class. task handler returns the internal
         * specific name of the task which is used in the next step.
         */
        ROS_DEBUG("%s Executing task '%s' ...", prefix.c_str(), task_to_run.task_info_.getName().c_str());

        // Load the task library
        loadTask(task_to_run);

        /*
         * Create an instance of the task based on the class name. a task .so file might
         * contain multiple classes, therefore path is not enough and specific name
         * must be used
         */
        instantiateTask(task_to_run);

        /*
         * MOVE the task info into the list of running tasks. If the move operation is done
         * after starting the task, the process will likely crash.
         */
        ROS_DEBUG("%s Moving the task into the 'running_tasks' vector ...", prefix.c_str());
        running_tasks_.push_back (std::move(task_to_run));

        start_attempted = true;

        //Start the task
        startTaskThread(running_tasks_.back(), arguments);
    }
    catch( error::ErrorStackUtil & e )
    {
        // Append the error to local ErrorStack
        e.forward( prefix );
        error_handler_.append(e);

        if( start_attempted )
        {
            try
            {
                stopTask(running_tasks_.back().task_info_.getName());
            }
            catch( error::ErrorStackUtil & e2 )
            {
                e2.forward( prefix );
                error_handler_.append(e2);
            }
        }
    }
}


/* * * * * * * * *
 *  stopTaskByID
 * * * * * * * * */

void TaskManager::stopTaskByID( TemotoID::ID task_id )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage( system_prefix_, this->class_name_, __func__);

    try
    {
        // Find the task by iterating through the list of running tasks
        for (auto it = this->running_tasks_.begin(); it != this->running_tasks_.end(); it++)
        {
            if ((*it).task_pointer_->getID() == task_id)
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

                return;
            }
        }

        ROS_DEBUG("%s Could not find any tasks with TID: %d", prefix.c_str(), task_id);
    }
    catch(class_loader::ClassLoaderException& e)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::CLASS_LOADER_FAIL,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + e.what(),
                                     ros::Time::now() );
    }
    catch(...)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::UNHANDLED,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Received an unhandled exception",
                                     ros::Time::now() );
    }
}


/* * * * * * * * *
 *  stopTaskByName
 *  TODO: when multiple tasks with the same name are found then do something more reasonable than
 *        just stopping the first one.
 * * * * * * * * */

void TaskManager::stopTaskByName( std::string task_name )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    try
    {
        // Find the task by iterating through the list of running tasks
        for (auto it = this->running_tasks_.begin(); it != this->running_tasks_.end(); it++)
        {
            // If a task was found then delete it
            if ((*it).task_info_.getName() == task_name)
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

                return;
            }
        }

        ROS_DEBUG("%s Could not find any tasks named: %s", prefix.c_str(), task_name.c_str());
    }
    catch(class_loader::ClassLoaderException& e)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::CLASS_LOADER_FAIL,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + e.what(),
                                     ros::Time::now() );
    }
    catch(...)
    {
        // Rethrow the exception
        throw error::ErrorStackUtil( coreErr::UNHANDLED,
                                     error::Subsystem::CORE,
                                     error::Urgency::HIGH,
                                     prefix + "Received an unhandled exception.",
                                     ros::Time::now() );
    }
}


/* * * * * * * * *
 *  STOP TASK
 * * * * * * * * */

void TaskManager::stopTask( std::string task_name, TemotoID::ID task_id)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    try
    {
        // If task_id is specified, then stop the task by ID
        if( task_id != TemotoID::UNASSIGNED_ID )
        {
            stopTaskByID( task_id );
            return;
        }

        // If ID was not specified, but name was, then stop the task by name
        else if( task_name != "" )
        {
            stopTaskByName( task_name );
            return;
        }
    }
    catch( error::ErrorStackUtil & e )
    {
        // Rethrow the exception
        e.forward( prefix );
        throw e;
    }

    // If nothing was specified, then throw error
    throw error::ErrorStackUtil( coreErr::UNSPECIFIED_TASK,
                                 error::Subsystem::CORE,
                                 error::Urgency::MEDIUM,
                                 prefix + "Task name and task ID unspecified.",
                                 ros::Time::now() );
}

/* * * * * * * * *
 *  START TASK SERVICE CALLBACK
 * * * * * * * * */

/*
bool TaskManager::startTaskCallback (temoto_2::startTask::Request& req,
                                     temoto_2::startTask::Response& res)
{
    ROS_DEBUG( "[%s/TaskManager::startTaskCallback] Received a request to start task '%s'", this->system_prefix__.c_str(), req.task_name.c_str());

    // Load the task
    ROS_DEBUG( "[%s/TaskManager::startTaskCallback] Loading the task", this->system_prefix__.c_str());
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
    ROS_DEBUG( "[%s/TaskManager::startTaskCallback] Creating an instance of the task", this->system_prefix__.c_str());

    if ( !instantiateTask(taskClassName) )
    {
        res.code = 1;
        res.message = "Unable to create an instance of the task";
        return true;
    }

    //Start the task
    ROS_DEBUG( "[%s/TaskManager::startTaskCallback] Starting the task", this->system_prefix__.c_str());

    if ( startTask(taskClassName, task.second) )
    {
        TaskManager.stopTask(taskClassName);
    }
}
*/

/* * * * * * * * *
 *  STOP TASK SERVICE CALLBACK
 * * * * * * * * */

bool TaskManager::stopTaskCallback( temoto_2::stopTask::Request& req,
                                    temoto_2::stopTask::Response& res)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    ROS_DEBUG( "%s Received a request to stop task '%s'", prefix.c_str(), req.name.c_str());

    try
    {
        stopTask(req.name, req.task_id);

        res.code = 0;
        res.message = "task stopped";
    }

    catch( error::ErrorStackUtil & e )
    {
        // Append the error to local ErrorStack
        e.forward( prefix );
        error_handler_.append(e);

        res.code = 1;
        res.message = "failed to stop the task";
    }

    return true;
}

/* * * * * * * * *
 *  stopTaskMsgCallback
 * * * * * * * * */

void TaskManager::stopTaskMsgCallback( temoto_2::StopTaskMsg msg )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage(system_prefix_, this->class_name_, __func__);

    ROS_DEBUG( "%s Received a request to stop a task (TID = %ld)", prefix.c_str(), msg.task_id);
    try
    {
         stopTask( "", msg.task_id );
    }
    catch( error::ErrorStackUtil & e )
    {
        // Append the error to local ErrorStack
        e.forward( prefix );
        error_handler_.append(e);
    }
}
