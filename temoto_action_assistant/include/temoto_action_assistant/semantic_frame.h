#ifndef TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_H
#define TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_H

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace temoto_action_assistant
{

// Forward declare the Subject and ActionDescriptor structure
struct ActionDescriptor;
struct Subject;

typedef std::shared_ptr<ActionDescriptor> ActionDescriptorPtr;
typedef std::vector<Subject> Subjects ;

/**
 * @brief Stores information about the data that is contained by a subject
 */
struct DataInstance
{
  /**
   * @brief Type of the data. Used for data parsing purposes
   */
  enum Type
  {
    STRING,
    TOPIC,
    NUMBER,
    POINTER
  };

  /// Map that translates Types to Strings
  std::map<DataInstance::Type, std::string> type_to_str = {{DataInstance::STRING, "string"},
                                                           {DataInstance::TOPIC, "topic"},
                                                           {DataInstance::NUMBER, "number"},
                                                           {DataInstance::POINTER, "pointer"}};

  /// Map that translates Strings to Types
  std::map<std::string, DataInstance::Type> str_to_type_ = {{"string", DataInstance::STRING},
                                                            {"topic", DataInstance::TOPIC},
                                                            {"number", DataInstance::NUMBER},
                                                            {"pointer", DataInstance::POINTER}};

  Type type_;

  /**
   * @brief getTypeStr
   * @return
   */
  std::string getTypeStr()
  {
    return type_to_str[type_];
  }

  DataInstance::Type setTypeByStr(std::string type_str)
  {
    type_ = str_to_type_[type_str];
    return type_;
  }
};

/**
 * @brief Contains information about the subject
 */
struct Subject
{
  /**
   * @brief Type of the Subject
   */
  enum Type
  {
    WHAT,
    WHERE,
    NUMERIC
  };

  /// Map that translates Types to Strings
  std::map<Subject::Type, std::string> type_to_str = {{Subject::WHAT, "what"},
                                                      {Subject::WHERE, "where"},
                                                      {Subject::NUMERIC, "numeric"}};

  /// Map that translates Strings to Types
  std::map<std::string, Subject::Type> str_to_type_ = {{"what", Subject::WHAT},
                                                       {"where", Subject::WHERE},
                                                       {"numeric", Subject::NUMERIC}};

  /// Type of the subject
  Type type_;

  /// Words contained by the subject.
  std::vector<std::string> words_;

  /// Data contained by the subject
  std::vector<DataInstance> data_;

  /**
   * @brief getTypeStr
   * @return
   */
  std::string getTypeStr()
  {
    return type_to_str[type_];
  }

  /**
   * @brief setTypeByStr
   * @param type_str
   * @return
   */
  Subject::Type setTypeByStr(std::string type_str)
  {
    type_ = str_to_type_[type_str];
    return type_;
  }
};

/**
 * @brief The Interface struct
 */
struct Interface
{
  /**
   * @brief The Type enum
   */
  enum Type
  {
    SYNCHRONOUS,
    ASYNCHRONOUS
  };

  /// Map that translates Types to Strings
  std::map<Interface::Type, std::string> type_to_str =
      {{Interface::ASYNCHRONOUS, "asynchronous"},
       {Interface::SYNCHRONOUS, "synchronous"}};

  /// Map that translates Strings to Types
  std::map<std::string, Interface::Type> str_to_type_ =
      {{"synchronous", Interface::SYNCHRONOUS},
       {"asynchronous", Interface::ASYNCHRONOUS}};


  /// Type of the interface
  Interface::Type type_;

  Subjects input_subjects_;
  Subjects output_subjects_;

  uint32_t id_;

  /**
   * @brief getTypeStr
   * @return
   */
  std::string getTypeStr()
  {
    return type_to_str[type_];
  }

  /**
   * @brief setTypeByStr
   * @param type_str
   * @return
   */
  Interface::Type setTypeByStr(std::string type_str)
  {
    type_ = str_to_type_[type_str];
    return type_;
  }
};

/**
 * @brief The ActionDescriptor struct
 */
struct ActionDescriptor
{
  std::string lexical_unit_;
  std::vector<Interface> interfaces_;
  std::string action_pkg_name_;
  std::string action_class_name_;
  std::string action_pkg_path_;
};

} // temoto_action_assistant
#endif
