#include "package_info/package_info.h"

/* * * * * * * * * * * *
 *     CONSTRUCTORS
 * * * * * * * * * * * */

PackageInfo::PackageInfo()
{
  // Fill array with initial reliability values of 0.5;
  reliabilities_.fill(0.8);
  reliability_average = 0.8;
  reliability_idx = 0;
}

PackageInfo::PackageInfo(std::string name, std::string type) : name_(name), type_(type)
{
  PackageInfo();
}

/* * * * * * * * * * * *
 *     SETTERS
 * * * * * * * * * * * */

// Set description
bool PackageInfo::setDescription(std::string description)
{
  this->description_ = description;
  return true;
}

// Set runnables
bool PackageInfo::setRunnables(std::map<std::string, std::string> runnables)
{
  this->runnables_ = runnables;
  return true;
}

// Set launchables
bool PackageInfo::setLaunchables(std::map<std::string, std::string> launchables)
{
  this->launchables_ = launchables;
  return true;
}

/* * * * * * * * * * * *
 *     GETTERS
 * * * * * * * * * * * */

// Get name
std::string PackageInfo::getName()
{
  return this->name_;
}

// Get type
std::string PackageInfo::getType()
{
  return this->type_;
}

// Get description
std::string PackageInfo::getDescription()
{
  return this->description_;
}


// Adds a reliability contribution to a moving average filter
void PackageInfo::adjustReliability(float reliability)
{
  reliability = std::max(std::min(reliability, 1.0f), 0.0f); // clamp into [0-1]
  ++reliability_idx %= reliabilities_.size(); // rolling index
  reliability_average -= reliabilities_[reliability_idx] / (float)reliabilities_.size();
  reliabilities_[reliability_idx % reliabilities_.size()] =
      reliability / (float)reliabilities_.size();
}

// Get runnables
std::map<std::string, std::string> PackageInfo::getRunnables()
{
  return this->runnables_;
}

// Get launchables
std::map<std::string, std::string> PackageInfo::getLaunchables()
{
  return this->launchables_;
}

/* * * * * * * * * * * *
 *     OTHER
 * * * * * * * * * * * */

// Add a runnable into the list of runnables
bool PackageInfo::addRunnable(std::pair<std::string, std::string> runnable)
{
  this->runnables_.insert(runnable);
  return true;
}

// Add a launchable into the list of launchables
bool PackageInfo::addLaunchable(std::pair<std::string, std::string> launchable)
{
  this->launchables_.insert(launchable);
  return true;
}

// Remove a runnable from the list of runnables
bool PackageInfo::removeRunnable(std::string runnable)
{
  this->runnables_.erase(runnable);
  return true;
}

// Remove a launchable from the list of launchables
bool PackageInfo::removeLaunchable(std::string launchable)
{
  this->launchables_.erase(launchable);
  return true;
}
