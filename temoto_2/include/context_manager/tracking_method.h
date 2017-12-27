#ifndef TRACKING_INFO_H
#define TRACKING_INFO_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <ctype.h>
#include <memory>                     // shared_ptr
#include "common/temoto_log_macros.h"
//#include "common/reliability.h"
#include <yaml-cpp/yaml.h>

#include <iostream>                   // TODO: remove

namespace context_manager
{

/**
 * @brief The FilterCategory enum
 */
enum class FilterCategory : int
{
  SENSOR,
  ALGORITHM
};

/**
 * @brief The Filter struct
 */
struct Filter
{
  std::string filter_category_;                       // Sensor or algorithm or ...
  std::string filter_type_;                           // Camera ... or ARtag detector ...
  std::set<std::string> required_input_topic_types_;  // The types of the topics that this filter requires
  std::set<std::string> required_output_topic_types_; // The types of the topics that this filter must publish

  /// add input topic type
  void addInputTopicType(std::string topic_type)
  {
    required_input_topic_types_.insert(topic_type);
  }

  /// add output topic type
  void addOutputTopicType(std::string topic_type)
  {
    required_output_topic_types_.insert(topic_type);
  }

  /// to string
  std::string toString() const
  {
    std::string str;
    str += "|_+_filter category: " + filter_category_ + "\n";
    str += "| |_filter type: " + filter_type_ + "\n";

    // Print out the input topics
    if (!required_input_topic_types_.empty())
    {
      str += "| |_required input topic types: ";
      for (auto& topic : required_input_topic_types_)
      {
        str += topic;
        if (topic != *std::prev(required_input_topic_types_.end()))
        {
          str += ", ";
        }
      }
      str += "\n";
    }

    // Print out the output topics
    if (!required_output_topic_types_.empty())
    {
      str += "| |_required output topic types: ";
      for (auto& topic : required_output_topic_types_)
      {
        str += topic;
        if (topic != *std::prev(required_output_topic_types_.end()))
        {
          str += ", ";
        }
      }
      str += "\n";
    }

    return str;
  }
};

/**
 * @brief operator ==
 * @param f1
 * @param f2
 * @return
 */
static bool operator==(const Filter& f1, const Filter& f2)
{
  // Check the category, type and topic types
  return f1.filter_category_ == f2.filter_category_ &&
         f1.filter_type_ == f2.filter_type_ &&
         f1.required_input_topic_types_ == f2.required_input_topic_types_ &&
         f1.required_output_topic_types_ == f2.required_output_topic_types_;
}

/**
 * @brief operator <<
 * @param out
 * @param f
 * @return
 */
static std::ostream& operator<<(std::ostream& out, const Filter& f)
{
    out << f.toString();

    return out;
}

/**
 * @brief The TrackerInfo class
 */
class TrackerInfo
{
public:

  /*
   * Getters
   */

  /// Get type
  std::string getType() const
  {
    return type_;
  }

  /// Get pipe
  std::vector<Filter> getPipe() const
  {
    return pipe_;
  }

  /// Get pipe size
  unsigned int getPipeSize() const
  {
    return pipe_.size();
  }

  /*
   * Setters
   */

  /// Set the pipe
  void setPipe(std::vector<Filter> pipe)
  {
    pipe_ = pipe;
  }

  /// Add filter
  void addFilter(Filter filter)
  {
    pipe_.push_back(filter);
  }

  std::string toString()
  {
    std::string str;
    str += "type: " + std::string("TODO") + "\n";

    for (auto& filter : pipe_)
    {
      str += filter.toString();

      if (&filter != &pipe_.back())
      {
        str += "| \n";
      }
    }

    return str;
  }

  /**
   * @brief operator ==
   * @param t1
   * @param t2
   * @return
   */
  friend bool operator==(const TrackerInfo& t1, const TrackerInfo& t2)
  {
    return t1.type_ == t2.type_ && t1.pipe_ == t2.pipe_;
  }

private:
  
  std::string type_;
  std::vector<Filter> pipe_;
  //Reliability reliability_;
};


/**
 * @brief TrackerInfoPtr
 */
typedef std::shared_ptr<TrackerInfo> TrackerInfoPtr;

/**
 * @brief TrackerInfoPtrs
 */
typedef std::vector<TrackerInfoPtr> TrackerInfoPtrs;

} // namespace context_manager

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                      YAML PARSER FOR TRACKER INFO CLASS
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace YAML
{
template <>
struct convert<context_manager::TrackerInfo>
{
  static Node encode(const context_manager::TrackerInfo& tracker_info)
  {
    Node method;
    std::vector<context_manager::Filter> pipe = tracker_info.getPipe();
    for (auto& filter : pipe)
    {
      // Encode the filter
      Node filter_node;
      filter_node["filter_category"] = filter.filter_category_;
      filter_node["filter_type"] = filter.filter_type_;

      // Encode the input topic types (if this filter has any)
      if (filter.required_input_topic_types_.size() != 0)
      {
        for (auto& topic_type : filter.required_input_topic_types_)
        {
          filter_node["input_topic_types"].push_back(topic_type);
        }
      }

      // Encode the output topic types (if this filter has any)
      if (filter.required_output_topic_types_.size() != 0)
      {
        for (auto& topic_type : filter.required_output_topic_types_)
        {
          filter_node["output_topic_types"].push_back(topic_type);
        }
      }

      // Push the filter
      method.push_back(filter_node);
    }

    return method;
  }

  static bool decode(const Node& node, context_manager::TrackerInfo& tracker_info)
  {
    // Check if the "node" is a map
    if (!node.IsMap())
    {
      return false;
    }

    // Get the pipe (sequence of filters) node
    YAML::Node filters_node = node["method"];

    // Iterate over each filter
    for (YAML::const_iterator filter_it = filters_node.begin(); filter_it != filters_node.end(); ++filter_it)
    {
      // Check if the filter is a map
      if (!filter_it->IsMap())
      {
        return false;
      }

      // Create an empty filter object and fill it
      context_manager::Filter filter;

      try
      {
        // TODO: Check if it is even a valid category
        filter.filter_category_ = (*filter_it)["filter_category"].as<std::string>();
        filter.filter_type_ = (*filter_it)["filter_type"].as<std::string>();
      }
      catch (YAML::InvalidNode e)
      {
        // Print out the error message
        // TODO: throw a proper error
        std::cout << "The filter node is either missing category or type\n";
        return false;
      }

      // Get the input topic types (if there are any)
      try
      {
        Node input_topics_node = (*filter_it)["input_topic_types"];
        for (YAML::const_iterator topics_it = input_topics_node.begin(); topics_it != input_topics_node.end(); ++topics_it)
        {
          filter.addInputTopicType(topics_it->as<std::string>());
        }
      }
      catch (YAML::InvalidNode e)
      {
        // REPORT OR DO SOMETHING
      }

      // Get the output topic types (if there are any)
      try
      {
        Node output_topics_node = (*filter_it)["output_topic_types"];
        for (YAML::const_iterator topics_it = output_topics_node.begin(); topics_it != output_topics_node.end(); ++topics_it)
        {
          filter.addOutputTopicType(topics_it->as<std::string>());
        }
      }
      catch (YAML::InvalidNode e)
      {
        // REPORT OR DO SOMETHING
      }

      tracker_info.addFilter(filter);
    }

    return true;
  }
};
}
#endif
