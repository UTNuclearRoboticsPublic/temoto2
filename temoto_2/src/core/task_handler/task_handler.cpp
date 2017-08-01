#include "core/task_handler/task_handler.h"

/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskHandler::TaskHandler( class_loader::MultiLibraryClassLoader * loader)
{
    loader_ = loader;
    //fProcessor_.setTaskToArgBook( &(this->taskToArgBook_) )
}


/* * * * * * * * *
 *  FIND TASK LOCAL
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string taskToFind)
{
    std::vector <TaskInfo> tasksFound;
    for (auto& task : this->tasksIndexed_)
    {
        if (taskToFind.compare(task.getName()) == 0)
        {
            tasksFound.push_back(task);
        }
    }
    return tasksFound;
}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string taskToFind, boost::filesystem::directory_entry basePath, int searchDepth)
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
                std::vector<TaskInfo> subTasksFound = TaskHandler::findTask( taskToFind, *itr, (searchDepth - 1));

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
        std::cout << "[DescriptionProcessor/findTask]: " << e.what() << '\n';
        return tasksFound;
    }

    catch(...)
    {
        std::cerr << "[DescriptionProcessor/findTask] Unhandled exception" << std::endl;
        return tasksFound;
    }
}

/* * * * * * * * *
 *  INDEX TASKS
 * * * * * * * * */

void TaskHandler::indexTasks (boost::filesystem::directory_entry basePath, int searchDepth)
{
    this->tasksIndexed_ = findTask ("", basePath, searchDepth);
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

std::string TaskHandler::loadTask(std::string taskName)
{
    // Create an empty map for storing tasks that were loaded
    std::vector<std::string> classes;

    // Find the task and get the "path" to its lib file
    std::string pathToLib =  findTask(taskName)[0].getLibPath();

    try
    {
        // Start loading a task library
        loader_->loadLibrary(pathToLib);
        classes = loader_->getAvailableClassesForLibrary<Task>(pathToLib);

        // Done loading
        ROS_DEBUG( "[TaskHandler/loadTask] Loaded %lu classes from %s", classes.size(), pathToLib.c_str() );
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("[TaskHandler/loadTask] ClassLoaderException: %s", e.what());
    }

    return classes[0];
}


/* * * * * * * * *
 *  INSTANTIATE TASK
 * * * * * * * * */

int TaskHandler::instantiateTask(std::string taskName)
{
    // Check if there is a task with this name
    bool taskFound = false;
    std::vector<std::string> loadedClasses = loader_->getAvailableClasses<Task>();

    for (std::string singleClass : loadedClasses)
    {
        if (singleClass.compare(taskName) == 0)
        {
            taskFound = true;
            break;
        }
    }

    // If the task was found, create an instance of it and call startTask method
    if (taskFound)
    {
        try
        {
            ROS_DEBUG( "[TaskHandler/instantiateTask] instatiating task: %s", taskName.c_str());
            runningTasks_.insert ( std::pair< std::string, boost::shared_ptr<Task> >(taskName, loader_->createInstance<Task>(taskName)) );
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("[TaskHandler/instantiateTask] ClassLoaderException: %s", e.what());
            return 1;
        }
    }

    else
    {
        ROS_DEBUG( "[TaskHandler/instantiateTask] task: '%s' was not found", taskName.c_str());
        return 1;
    }

    return 0;
}


/* * * * * * * * *
 *  START TASK WITHOUT ARGUMENTS
 * * * * * * * * */

int TaskHandler::startTask(std::string taskName)
{
    try
    {
        ROS_DEBUG( "[TaskHandler/startTask] starting task: %s", taskName.c_str());
        runningTasks_.at(taskName)->startTask();
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("[TaskHandler/startTask] ClassLoaderException: %s", e.what());
        return 1;
    }

    return 0;
}

/* * * * * * * * *
 *  START TASK WITH ARGUMENTS
 * * * * * * * * */

int TaskHandler::startTask(std::string taskName, std::vector<boost::any> arguments)
{
    // If the list is empty then ...
    if (arguments.empty())
    {
        startTask(taskName);
    }

    else
    {
        try
        {
            ROS_DEBUG( "[TaskHandler/startTask] starting task (with arguments): %s", taskName.c_str());
            runningTasks_.at(taskName)->startTask(0, arguments);
        }

        catch(class_loader::ClassLoaderException& e)
        {
            ROS_ERROR("[TaskHandler/startTask] ClassLoaderException: %s", e.what());
            return 1;
        }
    }

    return 0;
}


/* * * * * * * * *
 *  STOP TASK
 * * * * * * * * */

int TaskHandler::stopTask(std::string taskName)
{
    try
    {
        ROS_DEBUG( "[TaskHandler/stopTask] Stopping and erasing task: %s", taskName.c_str());
        runningTasks_.erase (runningTasks_.find(taskName));
    }

    catch(class_loader::ClassLoaderException& e)
    {
        ROS_ERROR("[TaskHandler/stopTask] ClassLoaderException: %s", e.what());
        return 1;
    }

    catch(...)
    {
        ROS_ERROR("[TaskHandler/stopTask] exception happened");
        return 1;
    }

    return 0;
}


/* * * * * * * * *
 *  UNLOAD LIB
 * * * * * * * * */

int TaskHandler::unloadTaskLib(std::string pathToLib)
{
    ROS_INFO( "[TaskHandler/unloadTaskLib] unloading library");
    loader_->unloadLibrary(pathToLib);
    return 0;
}
