#ifndef TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_H
#define TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_H

#include <string>
#include <vector>
#include <map>

namespace temoto_action_assistant
{

// Forward declare the Subject structure
struct Subject;
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

  Type type_;
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
};

/**
 * @brief The Interface struct
 */
struct Interface
{
  Subjects input_subjects_;
  Subjects output_subjects_;
  uint32_t id_;
};

/**
 * @brief The ActionDescriptor struct
 */
struct ActionDescriptor
{
  std::string lexical_unit_;
  std::vector<Interface> interfaces_;
  std::string action_pkg_name_;
  std::string action_pkg_path_;
};

} // temoto_action_assistant
#endif
