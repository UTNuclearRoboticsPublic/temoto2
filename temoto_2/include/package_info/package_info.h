#ifndef PACKAGE_INFO_H
#define PACKAGE_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr

class PackageInfo
{
public:
  /**
   * @brief PackageInfo
   */
  PackageInfo();

  PackageInfo(std::string name, std::string type);

  bool setDescription(std::string description);

  bool setRunnables(std::map<std::string, std::string> runnables);

  bool setLaunchables(std::map<std::string, std::string> launchables);

  /**
   * \brief Adjust reliability
   * \param reliability the new reliability contribution to the moving average filter. The value has
   * to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void adjustReliability(float reliability = 1.0);

  std::string getName();

  std::string getType();

  std::string getDescription();

  float getReliability() const
  {
    return reliability_average;
  };

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
   * @brief temoto_core::Reliability ratings of the package.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief temoto_core::Reliability moving average.
   */
  float reliability_average;

  /**
   * @brief temoto_core::Reliability rating of the package.
   */
  unsigned int reliability_idx;
};

typedef std::shared_ptr<PackageInfo> PackageInfoPtr;
#endif
