#ifndef HUMAN_CONTEXT_SERVICES_H
#define HUMAN_CONTEXT_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"
#include "temoto_2/LoadGesture.h"
#include "temoto_2/LoadSpeech.h"
#include "temoto_2/LoadTracker.h"
#include "temoto_2/AddObjects.h"

namespace context_manager
{
	namespace srv_name
	{
    const std::string MANAGER = "context_manager";
    const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync_1";
    const std::string GESTURE_SERVER = "load_gesture";
    const std::string SPEECH_SERVER = "load_speech";

    const std::string MANAGER_2 = "context_manager_2";
    const std::string SYNC_TOPIC_2 = "/temoto_2/"+MANAGER_2+"/sync_2";
    const std::string TRACKER_SERVER = "load_tracker";

    const std::string SERVER_ADD_OBJECTS = "add_objects";
  }
}

static bool operator==(const temoto_2::LoadTracker::Request& r1,
                       const temoto_2::LoadTracker::Request& r2)
{
    return( r1.detection_method == r2.detection_method);
}

static bool operator==(const temoto_2::LoadGesture::Request& r1,
		const temoto_2::LoadGesture::Request& r2)
{
    if (r1.gesture_specifiers.size() != r2.gesture_specifiers.size())
    {
        return false;
    }
    
    // compare elementwise
    auto it1 = r1.gesture_specifiers.begin();
    auto it2 = r2.gesture_specifiers.begin();
    while (it1 != r1.gesture_specifiers.end() && 
           it2 != r2.gesture_specifiers.end()) 
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


static bool operator==(const temoto_2::LoadSpeech::Request& r1,
		const temoto_2::LoadSpeech::Request& r2)
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
