#ifndef ALGORITHM_MANAGER_SERVICES_H
#define ALGORITHM_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_2/LoadAlgorithm.h"

namespace algorithm_manager
{
  // TODO: Change the srv_name to something more reasonable
  namespace srv_name
  {
    const std::string MANAGER = "algorithm_manager";
    const std::string SERVER = "load_algorithm";
    const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync";
  }
}

static bool operator==(const temoto_2::LoadAlgorithm::Request& r1
                     , const temoto_2::LoadAlgorithm::Request& r2)
{
  // Check the namespace, executable and name of the packate
  if (r1.algorithm_type != r2.algorithm_type ||
          r1.package_name != r2.package_name ||
          r1.executable != r2.executable)
  {
    return false;
  }

  // Check the size of input and output topics
  if (r1.input_topics.size() != r2.input_topics.size() ||
      r1.output_topics.size() != r2.output_topics.size())
  {
    return false;
  }

  // Check the input topics
  auto input_topics_2_copy = r2.input_topics;
  for (auto& input_topic_1 : r1.input_topics)
  {
    bool topic_found = false;
    for (auto it=input_topics_2_copy.begin(); it!=input_topics_2_copy.end(); it++)
    {
      if (input_topic_1.key == it->key &&
          input_topic_1.value == it->value)
      {
        topic_found = true;
        input_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
  }

  // Check the output topics
  auto output_topics_2_copy = r2.output_topics;
  for (auto& output_topic_1 : r1.output_topics)
  {
    bool topic_found = false;
    for (auto it=output_topics_2_copy.begin(); it!=output_topics_2_copy.end(); it++)
    {
      if (output_topic_1.key == it->key &&
          output_topic_1.value == it->value)
      {
        topic_found = true;
        output_topics_2_copy.erase(it);
        break;
      }
    }

    if (!topic_found)
    {
      return false;
    }
  }

  // The algorithm infos are equal
  return true;
}
#endif
