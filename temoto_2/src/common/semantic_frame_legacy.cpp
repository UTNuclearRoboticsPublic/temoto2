#include "common/semantic_frame_legacy.h"

TTP::Subjects toLegacySubjects(temoto_action_assistant::Subjects& ad_subjects)
{
  TTP::Subjects td_subjects;

  for (temoto_action_assistant::Subject& ad_subject : ad_subjects)
  {
    TTP::Subject td_subject;
    td_subject.words_ = ad_subject.words_;
    td_subject.type_ = ad_subject.getTypeStr();

    // Convert Data
    for (temoto_action_assistant::DataInstance ad_data_instance : ad_subject.data_)
    {
      TTP::Data td_data;
      td_data.type = ad_data_instance.getTypeStr();
      td_subject.data_.push_back(td_data);
    }

    td_subjects.push_back(td_subject);
  }

  return td_subjects;
}

TTP::TaskDescriptor toLegacyTaskDescriptor(temoto_action_assistant::ActionDescriptor action_descriptor)
{
  // Empty legacy task descriptor interfaces
  std::vector<TTP::TaskInterface> td_interfaces;

  // Convert interfaces
  for (auto& ad_interface : action_descriptor.interfaces_)
  {
    TTP::TaskInterface td_interface;

    // Convert the type
    td_interface.type_ = ad_interface.getTypeStr();

    // Convert input subjects
    td_interface.input_subjects_ = toLegacySubjects(ad_interface.input_subjects_);

    // Convert output subjects
    td_interface.output_subjects_ = toLegacySubjects(ad_interface.output_subjects_);

    td_interfaces.push_back(td_interface);
  }

  // Create the task descriptor
  TTP::TaskDescriptor task_descriptor(action_descriptor.lexical_unit_, td_interfaces);
  task_descriptor.setTaskPackageName(action_descriptor.action_pkg_name_);
  task_descriptor.setTaskClassName(action_descriptor.action_class_name_);

  return task_descriptor;
}
