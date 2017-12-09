#ifndef BASE_SUBSYSTEM_H
#define BASE_SUBSYSTEM_H

#include "temoto_error/temoto_error.h"

class BaseSubsystem
{
public:

  BaseSubsystem() = default;

  BaseSubsystem(BaseSubsystem& b)
    : subsystem_name_ (b.subsystem_name_)
    , subsystem_code_ (b.subsystem_code_)
    , log_group_ (b.log_group_)
    , error_handler_ (error::ErrorHandler(b.subsystem_code_, b.log_group_))
  {}

protected:

  error::Subsystem subsystem_code_;
  error::ErrorHandler error_handler_;
  std::string subsystem_name_;
  std::string class_name_;
  std::string log_group_;

  /**
   * @brief This function is used when the BaseSubsystem cannot be initialized
   * during the construction phase
   * @param b
   */
  void initializeBase(const BaseSubsystem* b)
  {
    subsystem_name_ = b->subsystem_name_;
    subsystem_code_ = b->subsystem_code_;
    log_group_ = b->log_group_;
    error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);
  }
};

#endif
