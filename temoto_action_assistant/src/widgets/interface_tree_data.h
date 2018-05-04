#ifndef TEMOTO_ACTION_ASSISTANT_INTERFACE_TREE_DATA_H
#define TEMOTO_ACTION_ASSISTANT_INTERFACE_TREE_DATA_H

#ifndef Q_MOC_RUN
#endif

#include "boost/any.hpp"

namespace temoto_action_assistant
{

/**
 * @brief // Metatype Class For Holding pointers to elements of the interfaces tree
 */
class InterfaceTreeData
{
public:

  /**
   * @brief The ElementType enum
   */
  enum ElementType
  {
    INTERFACE,
    INPUT,
    OUTPUT,
    SUBJECT,
    DATA
  };

  InterfaceTreeData(){}
  InterfaceTreeData(const InterfaceTreeData::ElementType type, boost::any payload)
  : type_(type),
    payload_(payload)
  {}

  virtual ~InterfaceTreeData(){}

  InterfaceTreeData::ElementType type_;
  boost::any payload_;
};

} // temoto_action_assistant namespace

// Declare the metatype
Q_DECLARE_METATYPE(temoto_action_assistant::InterfaceTreeData);
#endif
