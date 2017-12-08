#ifndef BASE_SUBSYSTEM_H
#define BASE_SUBSYSTEM_H

#include "base_error/base_error.h"

class BaseSubsystem
{
public:

  BaseSubsystem(){}

  BaseSubsystem(BaseSubsystem& b)
    : SUBSYSTEM_NAME_ (b.SUBSYSTEM_NAME_)
    , CLASS_NAME_ (b.CLASS_NAME_)
    , SUBSYSTEM_CODE_ (b.SUBSYSTEM_CODE_)
    , ERROR_HANDLER_ (error::ErrorHandler(b.SUBSYSTEM_CODE_))
  {}

protected:

  error::Subsystem SUBSYSTEM_CODE_;
  error::ErrorHandler ERROR_HANDLER_;

  std::string SUBSYS_NAME_;
  std::string CLASS_NAME_;

  /**
   * @brief This function is used when the BaseSubsystem cannot be initialized
   * during the construction phase
   * @param b
   */
  void initializeBase(Base* b)
  {
    SUBSYSTEM_NAME_ = b.SUBSYSTEM_NAME_;
    CLASS_NAME_ = b.CLASS_NAME_;
    SUBSYSTEM_CODE_ = b.SUBSYSTEM_CODE_;
    ERROR_HANDLER_.setSubsystem(b.SUBSYSTEM_CODE_);
  }
};

#endif
