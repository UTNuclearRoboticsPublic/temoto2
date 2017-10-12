#ifndef PACKAGE_INFO_H
#define PACKAGE_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>

class package_info
{
public:
  /**
   * @brief package_info
   */
  package_info();

  package_info(std::string name, std::string type);

  bool setDescription(std::string description);

  bool setRunnables(std::map<std::string, std::string> runnables);

  bool setLaunchables(std::map<std::string, std::string> launchables);
  
  void increaseReliability();
  
  void decreaseReliability();

  std::string getName();

  std::string getType();

  std::string getDescription();
  
  float getReliability();

  std::map<std::string, std::string> getRunnables();

  std::map<std::string, std::string> getLaunchables();

  bool addRunnable(std::pair<std::string, std::string> runnable);

  bool addLaunchable(std::pair<std::string, std::string> launchable);

  bool removeRunnable(std::string runnable);

  bool removeLaunchable(std::string launchable);

private:
  /**
   * @brief Name of the package
   */
  std::string name_;

  /**
   * @brief Type of the package, e.g., "Hand tracker", "LIDAR", etc.
   */
  std::string type_;

  /**
   * @brief A description of the package
   */
  std::string description_;

  /**
   * @brief Map of "rosrun" executables and published topics
   */
  std::map<std::string, std::string> runnables_;

  /**
   * @brief Map of "roslaunch" executables and published topics
   */
  std::map<std::string, std::string> launchables_;

  /**
   * @brief Reliability rating of the package.
   */
  float reliability;
};

#endif
