#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_context_manager/LoadSpeech.h"
#include "temoto_context_manager/GetNumber.h"
#include "temoto_context_manager/LoadTracker.h"
#include "temoto_context_manager/AddObjects.h"
#include "temoto_context_manager/TrackObject.h"

namespace temoto_context_manager
{
  namespace srv_name
  {
    const std::string MANAGER = "temoto_context_manager";
    const std::string SYNC_OBJECTS_TOPIC = "/temoto_context_manager/"+MANAGER+"/sync_objects";
    const std::string SYNC_TRACKED_OBJECTS_TOPIC= "/temoto_context_manager/"+MANAGER+"/sync_tracked_objects";
    const std::string SPEECH_SERVER = "load_speech";
    const std::string GET_NUMBER_SERVER = "get_number";
    const std::string TRACK_OBJECT_SERVER = "track_objects";

    const std::string MANAGER_2 = "temoto_context_manager_2";
    const std::string TRACKER_SERVER = "load_tracker";

    const std::string SERVER_ADD_OBJECTS = "add_objects";
  }
}

static bool operator==(const temoto_context_manager::GetNumber::Request& r1,
                       const temoto_context_manager::GetNumber::Request& r2)
{
  return( r1.requested_int == r2.requested_int);
}

/**
 * @brief operator ==
 * @param r1
 * @param r2
 * @return
 */
static bool operator==(const temoto_context_manager::LoadTracker::Request& r1,
                       const temoto_context_manager::LoadTracker::Request& r2)
{
  return( r1.detection_method == r2.detection_method);
}

/**
 * @brief operator ==
 * @param r1
 * @param r2
 * @return
 */
static bool operator==(const temoto_context_manager::TrackObject::Request& r1,
                       const temoto_context_manager::TrackObject::Request& r2)
{
    return( r1.object_name == r2.object_name);
}

static bool operator==(const temoto_context_manager::LoadSpeech::Request& r1,
		                   const temoto_context_manager::LoadSpeech::Request& r2)
{
    if (r1.speech_specifiers.size() != r2.speech_specifiers.size())
    {
        return false;
    }
    
    // compare elementwise
    auto it1 = r1.speech_specifiers.begin();
    auto it2 = r2.speech_specifiers.begin();
    while (it1 != r1.speech_specifiers.end() && 
           it2 != r2.speech_specifiers.end()) 
    {
        if( it1->dev != it2->dev ||
			it1->type != it2->type ||
            it1->package_name != it2->package_name ||
            it1->executable != it2->executable)
        {
            return false;
        }
        it1++;
        it2++;
    }
    return true;
}
#endif
