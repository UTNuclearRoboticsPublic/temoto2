#include "package_info/package_info.h"

/* * * * * * * * * * * *
 *     CONSTRUCTORS
 * * * * * * * * * * * */

package_info::package_info () {}

package_info::package_info (std::string name,
                            std::string type)
    : name_(name),
      type_(type)
{}


/* * * * * * * * * * * *
 *     SETTERS
 * * * * * * * * * * * */

// Set description
bool package_info::setDescription (std::string description)
{
    this->description_ = description;
    return true;
}

// Set runnables
bool package_info::setRunnables (std::map<std::string, std::string> runnables)
{
    this->runnables_ = runnables;
    return true;
}

// Set launchables
bool package_info::setLaunchables (std::map<std::string, std::string> launchables)
{
    this->launchables_ = launchables;
    return true;
}


/* * * * * * * * * * * *
 *     GETTERS
 * * * * * * * * * * * */

// Get name
std::string package_info::getName ()
{
    return this->name_;
}

// Get type
std::string package_info::getType ()
{
    return this->type_;
}

// Get description
std::string package_info::getDescription ()
{
    return this->description_;
}

// Get runnables
std::map<std::string, std::string> package_info::getRunnables ()
{
    return this->runnables_;
}

// Get launchables
std::map<std::string, std::string> package_info::getLaunchables ()
{
    return this->launchables_;
}


/* * * * * * * * * * * *
 *     OTHER
 * * * * * * * * * * * */

// Add a runnable into the list of runnables
bool package_info::addRunnable (std::pair<std::string, std::string> runnable)
{
    this->runnables_.insert(runnable);
    return true;
}

// Add a launchable into the list of launchables
bool package_info::addLaunchable (std::pair<std::string, std::string> launchable)
{
    this->launchables_.insert(launchable);
    return true;
}

// Remove a runnable from the list of runnables
bool package_info::removeRunnable (std::string runnable)
{
    this->runnables_.erase(runnable);
    return true;
}

// Remove a launchable from the list of launchables
bool package_info::removeLaunchable (std::string launchable)
{
    this->launchables_.erase(launchable);
    return true;
}
