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

TaskHandler::TaskHandler (std::string system_prefix,
                          class_loader::MultiLibraryClassLoader * loader)
    : system_prefix_(system_prefix),
      loader_(loader)
{  
    // Start the servers
    //this->start_task_server_ = n_.advertiseService (system_prefix_ + "/start_sensor", &TaskHandler::startTaskCallback, this);
    this->stop_task_server_ = n_.advertiseService (system_prefix_ + "/stop_task", &TaskHandler::stopTaskCallback, this);
}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string taskToFind, std::vector <TaskInfo>& tasks)
{
    std::vector <TaskInfo> tasksFound;
    for (auto& task : tasks)
    {
        if (taskToFind.compare(task.getName()) == 0)
        {
            tasksFound.push_back(task);
        }
    }
    return tasksFound;
}


/* * * * * * * * *
 *  FIND TASK LOCAL
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTaskLocal(std::string taskToFind)
{
    return findTask(taskToFind, this->tasksIndexed_);
}


/* * * * * * * * *
 *  FIND TASK RUNNING
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTaskRunning(std::string taskToFind)
{
    return findTask(taskToFind, this->running_tasks_);
}


/* * * * * * * * *
 *  FIND TASK FROM FILESYSTEM
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTaskFilesys(std::string taskToFind, boost::filesystem::directory_entry basePath, int searchDepth)
{
    boost::filesystem::path current_dir (basePath);
    boost::filesystem::directory_iterator end_itr;
    std::vector <TaskInfo> tasksFound;

    try
    {
        // Start looking the files inside current directory
        for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
        {

            // if its a directory and depth limit is not there yet, go inside it
            if ( boost::filesystem::is_directory(*itr) & (searchDepth > 0) )
            {
                std::vector<TaskInfo> subTasksFound = findTaskFilesys( taskToFind, *itr, (searchDepth - 1));

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
                                    "[TaskHandler/findTask] FORWARDING");

                    this->errorHandler_.append(e);
                }
            }
        }
        return tasksFound;
    }

    catch (std::exception& e)
    {
        std::cout << "[TaskHandler::findTask]: " << e.what() << '\n';
        return tasksFound;
    }

    catch(...)
    {
        std::cerr << "[TaskHandler::findTask] Unhandled exception" << std::endl;
        return tasksFound;
    }
}

/* * * * * * * * *
 *  INDEX TASKS
 * * * * * * * * */

void TaskHandler::indexTasks (boost::filesystem::directory_entry basePath, int searchDepth)
{
    this->tasksIndexed_ = findTaskFilesys ("", basePath, searchDepth);
}


/* * * * * * * * *
 *  GET THE INDEX TASKS
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::getIndexedTasks()
{
    return this->tasksIndexed_;
}


/* * * * * * * * *
 *  LOAD TASK
 * * * * * * * * */

bool TaskHandler::loadTask(TaskInfo& task)
{
    // Create an empty vector for storing the class names (currently a task
    // could have multiple tasks inside it. but it sounds TROUBLESOME
    std::vector<std::string> classes;

    // Get the "path" to its lib file
    std::string pathToLib =  task.getLibPath();

    try
    {
        // Start loading a task library
        loader_->loadLibrary(pathToLib);
        classes = loader_->getAvailableClassesForLibrary<Task>(pathToLib);

        // Done loading
        ROS_DEBUG( "[%s/TaskHandler::loadTask] Loaded %lu classes from %s", this->system_prefix_.c_str(), classes.size(), pathToLib.c_str() );

        // Add the name of the class
        task.class_name_ = classes[0];

        return true;
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("[%s/TaskHandler::loadTask] ClassLoaderException: %s", this->system_prefix_.c_str(), e.what());
        return false;
    }

    return false;
}


/* * * * * * * * *
 *  INSTANTIATE TASK
 * * * * * * * * */

bool TaskHandler::instantiateTask(TaskInfo& task)
{
    std::string taskClassName = task.getClassName();

    // First check that the task has a "class name"
    if (taskClassName.empty())
    {
        ROS_ERROR( "[%s/TaskHandler::instantiateTask] Unset 'class name'", this->system_prefix_.c_str() );
        return false;
    }

    // Check if there is a class with this name
    bool task_class_found = false;
    std::vector<std::string> loadedClasses = loader_->getAvailableClasses<Task>();

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
            ROS_DEBUG( "[%s/TaskHandler::instantiateTask] instatiating task: %s", this->system_prefix_.c_str(), taskClassName.c_str());
            task.task_pointer_ = loader_->createInstance<Task>(taskClassName);

            // Push the task info into the list of running tasks
            running_tasks_.push_back (task);
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("[%s/TaskHandler::instantiateTask] ClassLoaderException: %s", this->system_prefix_.c_str(), e.what());
            return false;
        }
    }
    else
    {
        ROS_DEBUG( "[%s/TaskHandler::instantiateTask] task: '%s' was not found", this->system_prefix_.c_str(), taskClassName.c_str());
        return false;
    }

    return true;
}


/* * * * * * * * *
 *  START TASK WITHOUT ARGUMENTS
 * * * * * * * * */
// TODO: CHECK IF THE TASK EVEN EXISTS IN THE LIST OF RUNNING TASKS

bool TaskHandler::startTask(TaskInfo& task)
{
    try
    {
        ROS_DEBUG( "[TaskHandler/startTask] starting task: %s", task.getName().c_str());
        task.task_pointer_->startTask();
        return true;
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("[TaskHandler/startTask] ClassLoaderException: %s", e.what());
        return false;
    }

    return false;
}

/* * * * * * * * *
 *  START TASK WITH ARGUMENTS
 *
 *  TODO: * check the running tasks lists before doing anything
 * * * * * * * * */

bool TaskHandler::startTask(TaskInfo& task, std::vector<boost::any> arguments)
{
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
            ROS_DEBUG( "[TaskHandler/startTask] starting task (with arguments): %s", task.getName().c_str());
            task.task_pointer_->startTask(0, arguments);
            return true;
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("[TaskHandler/startTask] ClassLoaderException: %s", e.what());
            return false;
        }
    }

    return false;
}


/* * * * * * * * *
 *  STOP TASK
 *
 *  TODO: when multiple tasks with the same name are found then do something more reasonable than
 *        just stopping the first one.
 * * * * * * * * */

bool TaskHandler::stopTask(std::string taskName)
{
    // Find the task by iterating through the list of running tasks
    for (auto it = this->running_tasks_.begin(); it != this->running_tasks_.end(); it++)
    {
        // If a task was found then delete it
        if ((*it).getName().compare(taskName) == 0)
        {
            try
            {
                ROS_DEBUG( "[%s/TaskHandler::stopTask] Stopping and erasing task: %s", this->system_prefix_.c_str(), (*it).getName().c_str());
                running_tasks_.erase (it);
                return true;
            }

            catch(class_loader::ClassLoaderException& e)
            {
                ROS_ERROR("[%s/TaskHandler::stopTask] ClassLoaderException: %s", this->system_prefix_.c_str(), e.what());
                return false;
            }

            catch(...)
            {
                ROS_ERROR("[%s/TaskHandler::stopTask] exception happened", this->system_prefix_.c_str());
                return false;
            }
        }
    }

    ROS_ERROR("[%s/TaskHandler::stopTask] exception happened", this->system_prefix_.c_str());
    return false;
}


/* * * * * * * * *
 *  UNLOAD LIB
 * * * * * * * * */

bool TaskHandler::unloadTaskLib(std::string pathToLib)
{
    ROS_INFO( "[TaskHandler/unloadTaskLib] unloading library");
    loader_->unloadLibrary(pathToLib);
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
