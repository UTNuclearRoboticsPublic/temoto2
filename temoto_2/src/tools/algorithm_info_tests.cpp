/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* This node is used for testing and developing of the algorithm
* info container
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "ros/package.h"
#include "algorithm_manager/algorithm_info.h"

#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <ros/serialization.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "temoto_2/RMPResponse.h"
#include "temoto_2/ConfigSync.h"


namespace ser = ros::serialization;

namespace context_manager
{

class AlgorithmTest
{
public:

    AlgorithmTest()
    {
        std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

        // Read the algorithms for this manager.
        std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                                    common::getTemotoNamespace() + ".yaml";
        std::ifstream in(yaml_filename);
        YAML::Node config = YAML::Load(in);

        if (config["Algorithms"])
        {
          local_algorithms_ = parseAlgorithms(config);



          for (auto& s : local_algorithms_)
          {
            TEMOTO_INFO("%s Added algorithm: '%s'.", prefix.c_str(), s->getName().c_str());
          }
          // notify other managers about our algorithms
          // advertiseLocalAlgorithms();
        }
        else
        {
          TEMOTO_WARN("%s Failed to read '%s'. Verify that the file exists and the sequence of algorithms "
                      "is listed under 'Algorithms' node.",
                      prefix.c_str(), yaml_filename.c_str());
        }

        // Print out the algorithms
        for (auto& algorithm : local_algorithms_)
        {
            std::cout << algorithm->toString() << std::endl;
        }

        YAML::Node test(*local_algorithms_.back());

        std::cout << Dump(test);
    }

private:

    std::string log_class_ = "AlgorithmTest";
    std::string log_subsys_ = "context_manager";
    std::string log_group_ = "context_manager";

    AlgorithmInfoPtrs local_algorithms_;

    // parseAlgorithms
    AlgorithmInfoPtrs parseAlgorithms(const YAML::Node& config)
    {
        std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
        std::vector<AlgorithmInfoPtr> algorithms;

        //  TEMOTO_INFO("%s CONFIG NODE:%d %s", prefix.c_str(), config.Type(), Dump(config).c_str());
        if (!config.IsMap())
        {
            // TODO Throw
            TEMOTO_WARN("%s Unable to parse 'algorithms' key from config.", prefix.c_str());
            return algorithms;
        }

        YAML::Node algorithms_node = config["Algorithms"];

        // Node throws
        TEMOTO_INFO("%s algorithms NODE:%d", prefix.c_str(), algorithms_node.Type());

        if (!algorithms_node.IsSequence())
        {
            TEMOTO_WARN("%s The given config does not contain sequence of algorithms.", prefix.c_str());
            // TODO Throw
            return algorithms;
        }



        TEMOTO_INFO("%s Parsing %lu algorithms.", prefix.c_str(), algorithms_node.size());

        // go over each algorithm node in the sequence
        for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
        {
            if (!node_it->IsMap())
            {
                TEMOTO_ERROR("%s Unable to parse the algorithm. Parameters in YAML have to be specified in "
                           "key-value pairs.",
                           prefix.c_str());
                continue;
            }

            try
            {
                AlgorithmInfo algorithm = node_it->as<AlgorithmInfo>();
                if (std::count_if(algorithms.begin(),
                                algorithms.end(),
                                [&](const AlgorithmInfoPtr& s) { return *s == algorithm; }) == 0)
                {
                    // OK, this is unique algorithm information, add it to the algorithms vector.
                    algorithms.emplace_back(std::make_shared<AlgorithmInfo>(algorithm));
                }
                else
                {
                    TEMOTO_WARN("%s Ignoring duplicate of algorithm '%s'.", prefix.c_str(),
                                algorithm.getName().c_str());
                }
            }
            catch (YAML::TypedBadConversion<AlgorithmInfo> e)
            {
                TEMOTO_WARN("%s Failed to parse AlgorithmInfo from config.", prefix.c_str());
                continue;
            }
        }
        return algorithms;
    }
};

} // context_manager namespace


void conf_sub_cb(temoto_2::ConfigSync conf_msg)
{
  temoto_2::RMPResponse my_value_back;

  uint32_t serial_size_back = conf_msg.payload.size();
  boost::shared_array<uint8_t> buffer_back(new uint8_t[serial_size_back]);

  // Fill buffer with the serialized payload
  for (int i=0; i<serial_size_back; i++)
  {
    (buffer_back.get())[i] = conf_msg.payload[i];
  }
  std::cout << std::endl;

  // Convert the serialized payload to msg
  ser::IStream stream_back(buffer_back.get(), serial_size_back);
  ser::deserialize(stream_back, my_value_back);

  std::cout << my_value_back << std::endl;
}


// main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm_info_tests");
  ros::NodeHandle n;

  ros::Publisher conf_pub = n.advertise<temoto_2::ConfigSync>("conf_test", 10);
  ros::Subscriber conf_sub = n.subscribe("conf_test", 10, conf_sub_cb);

  temoto_2::RMPResponse my_value;
  my_value.code = 34;
  my_value.message = "tervist";
  my_value.resource_id = 987;

  uint32_t serial_size = ros::serialization::serializationLength(my_value);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ser::OStream stream(buffer.get(), serial_size);
  ser::serialize(stream, my_value);

  // Create a byte array
  std::vector<uint8_t> byte_array_msg;

  // Fill out the byte array
  for (int i=0; i<serial_size; i++)
  {
    uint8_t msg;
    msg = (buffer.get())[i];
    byte_array_msg.push_back(msg);
  }

  // Create a config sync message
  temoto_2::ConfigSync config_sync_msg;
  config_sync_msg.action = "ADD";
  config_sync_msg.temoto_namespace = "test43test";
  config_sync_msg.payload = byte_array_msg;

  // Publish the message
  conf_pub.publish(config_sync_msg);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
