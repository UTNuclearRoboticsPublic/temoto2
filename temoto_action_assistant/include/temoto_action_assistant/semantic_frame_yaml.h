#ifndef TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_YAML_H
#define TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_YAML_H

#include "semantic_frame.h"
#include "yaml-cpp/yaml.h"
#include <iostream>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                      YAML PARSER FOR ACTION DESCRIPTOR STRUCT
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace YAML
{
template <>
struct convert<temoto_action_assistant::ActionDescriptor>
{
  static Node encode(const temoto_action_assistant::ActionDescriptor& action_descriptor)
  {
    // Action descriptor yaml node
    Node action_descriptor_node;
    action_descriptor_node["lexical_unit"] = action_descriptor.lexical_unit_;
    action_descriptor_node["package_name"] = action_descriptor.action_pkg_name_;
    action_descriptor_node["class_name"]   = action_descriptor.action_class_name_;
    action_descriptor_node["package_path"] = action_descriptor.action_pkg_path_;

    // Interfaces yaml node
    Node interfaces_node;
    std::vector<temoto_action_assistant::Interface> interfaces = action_descriptor.interfaces_;

    // Add interfaces to the interfaces node
    for (temoto_action_assistant::Interface& interface : interfaces)
    {
      // Interface node
      Node interface_node;
      interface_node["interface"] = std::to_string(interface.id_);
      interface_node["type"] = interface.getTypeStr();

      // Check if the interface contains an input
      if (!interface.input_subjects_.empty())
      {
        // TODO: Throw an error, because each interface must contain an input
      }

      // Encode the input of the interface
      for (temoto_action_assistant::Subject& subject: interface.input_subjects_)
      {
        Node subject_node;
        subject_node["type"] = subject.getTypeStr();
        subject_node["word"] = subject.words_[0];

        // Add the data
        for (temoto_action_assistant::DataInstance& data_instance : subject.data_)
        {
          subject_node["data"].push_back(data_instance.getTypeStr());
        }

        interface_node["input_subjects"].push_back(subject_node);
      }

      // Encode the output of the interface
      for (temoto_action_assistant::Subject& subject: interface.output_subjects_)
      {
        Node subject_node;
        subject_node["type"] = subject.getTypeStr();
        subject_node["word"] = subject.words_[0];

        // Add the data
        for (temoto_action_assistant::DataInstance& data_instance : subject.data_)
        {
          subject_node["data"].push_back(data_instance.getTypeStr());
        }

        interface_node["output_subjects"].push_back(subject_node);
      }

      // Push the interface
      action_descriptor_node["interfaces"].push_back(interface_node);
    }

    return action_descriptor_node;
  }

  static bool decode(const Node& node, temoto_action_assistant::ActionDescriptor& action_descriptor)
  {
    // Check if the "node" is a map
    if (!node.IsMap())
    {
      return false;
    }

    // Get the lexical unit
    action_descriptor.lexical_unit_      = node["lexical_unit"].as<std::string>();
    action_descriptor.action_pkg_name_   = node["package_name"].as<std::string>();
    action_descriptor.action_class_name_ = node["class_name"].as<std::string>();
    action_descriptor.action_pkg_path_   = node["package_path"].as<std::string>();

    // Get the interfaces
    YAML::Node interfaces_node = node["interfaces"];
    for (YAML::const_iterator interface_node_it = interfaces_node.begin();
         interface_node_it != interfaces_node.end();
         ++interface_node_it)
    {
      // Check if the filter is a map
      if (!interface_node_it->IsMap())
      {
        return false;
      }

      // Create an empty interface instance and fill it
      temoto_action_assistant::Interface interface;
      interface.id_ = (*interface_node_it)["interface"].as<int>();
      interface.setTypeByStr((*interface_node_it)["type"].as<std::string>());

      /*
       * Get the input subjects
       */
      YAML::Node input_subjects_node = (*interface_node_it)["input_subjects"];
      if (input_subjects_node)
      {
        // Extract the input subjects
        for (YAML::const_iterator input_subject_node_it = input_subjects_node.begin();
             input_subject_node_it != input_subjects_node.end();
             ++input_subject_node_it)
        {
          // Create an empty subject
          temoto_action_assistant::Subject input_subject;
          input_subject.setTypeByStr((*input_subject_node_it)["type"].as<std::string>());
          input_subject.words_.push_back((*input_subject_node_it)["word"].as<std::string>());

          // Get the data
          YAML::Node data_node = (*input_subject_node_it)["data"];

          if (data_node)
          {
            for (YAML::const_iterator data_instance_node_it = data_node.begin();
               data_instance_node_it != data_node.end();
               ++data_instance_node_it)
            {
              temoto_action_assistant::DataInstance data_instance;
              data_instance.setTypeByStr((*data_instance_node_it).as<std::string>());
              input_subject.data_.push_back(data_instance);
            }
          }

          interface.input_subjects_.push_back(input_subject);
        }
      }
      else
      {
        return false;
      }

      /*
       * Get the output subjects
       */
      YAML::Node output_subjects_node = (*interface_node_it)["output_subjects"];
      if (output_subjects_node)
      {
        // Extract the output subjects
        for (YAML::const_iterator output_subject_node_it = output_subjects_node.begin();
             output_subject_node_it != output_subjects_node.end();
             ++output_subject_node_it)
        {
          // Create an empty subject
          temoto_action_assistant::Subject subject;
          subject.setTypeByStr((*output_subject_node_it)["type"].as<std::string>());
          subject.words_.push_back((*output_subject_node_it)["word"].as<std::string>());

          // Get the data
          YAML::Node data_node = (*output_subject_node_it)["data"];

          // Check if there is any data
          if (data_node)
          {
            // Get the data
            for (YAML::const_iterator data_instance_node_it = data_node.begin();
                 data_instance_node_it != data_node.end();
                 ++data_instance_node_it)
            {
              temoto_action_assistant::DataInstance data_instance;
              data_instance.setTypeByStr((*data_instance_node_it).as<std::string>());
              subject.data_.push_back(data_instance);
            }
          }

          interface.output_subjects_.push_back(subject);
        }
      }

      // Add the interface
      action_descriptor.interfaces_.push_back(interface);
    }

    return true;
  }
};
}


#endif
