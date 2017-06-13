#include "core/task_handler/task_handler.h"

/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskHandler::TaskHandler( class_loader::MultiLibraryClassLoader * loader)
{
    loader_ = loader;
    //langProcessor_.setTaskToArgBook( &(this->taskToArgBook_) )
}


/* * * * * * * * *
 *  FIND TASK LOCAL
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string taskToFind)
{

}


/* * * * * * * * *
 *  FIND TASK
 * * * * * * * * */

std::vector <TaskInfo> TaskHandler::findTask(std::string taskToFind, boost::filesystem::directory_entry basePath, int searchDepth)
{
    boost::filesystem::path current_dir (basePath);
    boost::filesystem::directory_iterator end_itr;
    std::vector <TaskInfo> tasksFound = {};

    try
    {
        // Start looking the files inside current directory
        for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
        {

            // if its a directory and depth limit is not there yet, go inside it
            if ( boost::filesystem::is_directory(*itr) & (searchDepth > 0) )
            {
                std::vector<TaskInfo> subTasksFound = TaskHandler::findTask( taskToFind, *itr, (searchDepth - 1));

                // Append if it not empty
                if ( !subTasksFound.empty() )
                {
                    tasksFound.insert(std::end(tasksFound), std::begin(subTasksFound), std::end(subTasksFound));
                }
            }

            // if its a file and matches the desc file name, search the file
            else if ( boost::filesystem::is_regular_file(*itr) &
                      ((*itr).path().filename().compare(descriptionFile) == 0) )
            {
// HERE!                int res = processDesc (taskType, (*itr).path().string());

                if ( res == 0)
                {
                    std::cout << "Pushing: " << (*itr).path().string() << std::endl;
                    pathsFound.push_back( (*itr).path().string() );
                }

            }
        }
        return tasksFound;
    }

    catch(...)
    {
        std::cerr << "[DescriptionProcessor/findTask] Unhandled exception" << std::endl;
        return tasksFound;
    }





    // Create an empty map for storing tasks that were found
    std::map<std::string, std::string> foundTasks;

    this->descProcessor_.

    else
    {
        ROS_DEBUG( "[TaskHandler/findTask] didnt find any tasks named: %s", taskToFind.c_str());
    }

    return foundTasks;
}


/* * * * * * * * *
 *  LOAD TASK
 * * * * * * * * */

std::string TaskHandler::loadTask(std::string pathToLib)
{
    // Create an empty map for storing tasks that were loaded
    std::vector<std::string> classes;

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
