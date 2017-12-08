#ifndef HUMAN_CONTEXT_SERVICES_H
#define HUMAN_CONTEXT_SERVICES_H

#include <string>
#include "rmp/resource_manager_services.h"
#include "temoto_2/LoadGesture.h"
#include "temoto_2/LoadSpeech.h"
#include "temoto_2/AddObject.h"

namespace context_manager
{
	namespace srv_name
	{
    const std::string MANAGER = "context_manager";
    const std::string GESTURE_SERVER = "load_gesture";
    const std::string SPEECH_SERVER = "load_speech";
    const std::string SERVER_ADD_OBJECT = "add_object";
    const std::string TRACKING_SERVER = "load_tracker";
    const std::string SYNC_TOPIC = "/temoto_2/"+MANAGER+"/sync";
  }
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
