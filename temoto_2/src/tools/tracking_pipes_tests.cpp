/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* This node is used for testing and developing of the algorithm
* info container
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "ros/package.h"
#include "context_manager/tracking_method.h"

#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace ser = ros::serialization;

namespace context_manager
{

class TrackingPipesTest
{
public:

  TrackingPipesTest()
  {

    // Read the algorithms for this manager.
    std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                                "tracking_methods.yaml";

    // Parse the trackes
    parseTrackers(yaml_filename);

    // Print out the trackers
    for (auto& tracker_category : trackers_)
    {
      std::cout << "CATEGORY: " << tracker_category.first << std::endl;
      for (auto& tracking_method : tracker_category.second)
      {
        std::cout << tracking_method.toString() << std::endl;
      }
    }
  }

private:

  std::map<std::string, std::vector<context_manager::TrackerInfo>> trackers_;

  /**
   * @brief parseTrackers
   * @param config_path
   */
  void parseTrackers(std::string config_path)
  {
    // Read in the config file
    std::ifstream in(config_path);
    YAML::Node config = YAML::Load(in);

    // Check if it is a map
    if (!config.IsMap())
    {
      // TODO Throw
      std::cout << " throw throw throw \n";
      return;
    }

    // Iterate over different tracker categories (hand trackers, artag trackers, ...)
    for (YAML::const_iterator tracker_type_it = config.begin(); tracker_type_it != config.end(); ++tracker_type_it)
    {
      // Each category must contain a sequence of tracking methods
      if (!tracker_type_it->second.IsSequence())
      {
        // TODO Throw
        std::cout << " throw TODO throw TODO \n";
        return;
      }

      // Get the category of the tracker
      std::string tracker_category = tracker_type_it->first.as<std::string>();

      // Iterate over different tracking methods within the given category
      for (YAML::const_iterator method_it = tracker_type_it->second.begin();
           method_it != tracker_type_it->second.end();
           ++method_it)
      {
        try
        {
          // Convert the tracking method yaml description into TrackerInfo
          context_manager::TrackerInfo tracker_info = method_it->as<context_manager::TrackerInfo>();

          // Add the tracking method into the map of locally known trackers
          trackers_[tracker_category].push_back(tracker_info);

          // TODO: Print via TEMOTO_DEBUG
          // std::cout << tracker_info.toString() << std::endl;
        }
        catch (YAML::InvalidNode e)
        {
          // print out the error message
          std::cout << "Conversion failed: " << e.what() << std::endl;
        }
      }
    }
  }

//    AlgorithmInfoPtrs local_algorithms_;

//    // parseAlgorithms
//    AlgorithmInfoPtrs parseAlgorithms(const YAML::Node& config)
//    {
//        std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
//        std::vector<AlgorithmInfoPtr> algorithms;

//        //  TEMOTO_INFO("%s CONFIG NODE:%d %s", prefix.c_str(), config.Type(), Dump(config).c_str());
//        if (!config.IsMap())
//        {
//            // TODO Throw
//            TEMOTO_WARN("%s Unable to parse 'algorithms' key from config.", prefix.c_str());
//            return algorithms;
//        }

//        YAML::Node algorithms_node = config["Algorithms"];

//        // Node throws
//        TEMOTO_INFO("%s algorithms NODE:%d", prefix.c_str(), algorithms_node.Type());

//        if (!algorithms_node.IsSequence())
//        {
//            TEMOTO_WARN("%s The given config does not contain sequence of algorithms.", prefix.c_str());
//            // TODO Throw
//            return algorithms;
//        }



//        TEMOTO_INFO("%s Parsing %lu algorithms.", prefix.c_str(), algorithms_node.size());

//        // go over each algorithm node in the sequence
//        for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
//        {
//            if (!node_it->IsMap())
//            {
//                TEMOTO_ERROR("%s Unable to parse the algorithm. Parameters in YAML have to be specified in "
//                           "key-value pairs.",
//                           prefix.c_str());
//                continue;
//            }

//            try
//            {
//                AlgorithmInfo algorithm = node_it->as<AlgorithmInfo>();
//                if (std::count_if(algorithms.begin(),
//                                algorithms.end(),
//                                [&](const AlgorithmInfoPtr& s) { return *s == algorithm; }) == 0)
//                {
//                    // OK, this is unique algorithm information, add it to the algorithms vector.
//                    algorithms.emplace_back(std::make_shared<AlgorithmInfo>(algorithm));
//                }
//                else
//                {
//                    TEMOTO_WARN("%s Ignoring duplicate of algorithm '%s'.", prefix.c_str(),
//                                algorithm.getName().c_str());
//                }
//            }
//            catch (YAML::TypedBadConversion<AlgorithmInfo> e)
//            {
//                TEMOTO_WARN("%s Failed to parse AlgorithmInfo from config.", prefix.c_str());
//                continue;
//            }
//        }
//        return algorithms;
//    }
};

} // context_manager namespace

// main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_pipe_tests");
  ros::NodeHandle n;

  context_manager::TrackingPipesTest tpt;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
