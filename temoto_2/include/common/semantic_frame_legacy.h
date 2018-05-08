#ifndef TEMOTO_SEMANTIC_FRAME_LEGACY_H
#define TEMOTO_SEMANTIC_FRAME_LEGACY_H

#include "TTP/task_descriptor.h"
#include "temoto_action_assistant/semantic_frame.h"

/**
 * @brief Converts the new version of "task descriptor" to legacy version
 * @param action_descriptor
 * @return
 */
TTP::TaskDescriptor toLegacyTaskDescriptor(temoto_action_assistant::ActionDescriptor action_descriptor);

#endif
