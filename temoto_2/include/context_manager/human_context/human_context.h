#include "core/common.h"
#include "common/temoto_id.h"
#include "temoto_2/getGestures.h"
#include "temoto_2/getSpeech.h"
#include "temoto_2/stopAllocatedServices.h"
#include "temoto_2/LoadSensor.h"
#include "temoto_2/UnloadResource.h"
#include "temoto_2/gestureSpecifier.h"
#include "temoto_2/speechSpecifier.h"
#include "rmp/resource_manager.h"

class HumanContext
{
public:

    HumanContext();

private:
    ros::NodeHandle n_;

    TemotoID::IDManager id_manager_;

    std::vector <temoto_2::getGestures> setupGestureActive_;
    std::vector <temoto_2::getSpeech> setupSpeechActive_;

    // Define the basic services
    ros::ServiceServer gestureServer_;
    ros::ServiceServer speechServer_;
    ros::ServiceServer stop_allocated_services_;
    // ... And other interesting servers

	rmp::ResourceManager<HumanContext> resource_manager_;

    /**
     * @brief Service that sets up a gesture publisher
     * @param A gesture specifier message
     * @param Returns a topic where the requested gesture messages
     * are going to be published
     * @return
     */
    bool setup_gesture_cb (temoto_2::getGestures::Request &req,
                           temoto_2::getGestures::Response &res);

    /**
     * @brief Service that sets up a speech publisher
     * @param A gesture specifier message
     * @param Returns a topic where the requested gesture messages
     * are going to be published
     * @return
     */
    bool setup_speech_cb (temoto_2::getSpeech::Request &req,
                          temoto_2::getSpeech::Response &res);

    /**
     * @brief stopAllocatedServices
     * @param req
     * @param res
     * @return
     */
    bool stopAllocatedServices (temoto_2::stopAllocatedServices::Request& req,
                                temoto_2::stopAllocatedServices::Response& res);

    /**
     * @brief Function that compares the speech requests
     * @param req
     * @param reqLocal
     * @return
     */
    bool compareSpeechRequest (temoto_2::getSpeech::Request &req,
                               temoto_2::getSpeech::Request &reqLocal) const;

    bool compareGestureRequest (temoto_2::getGestures::Request &req,
                                temoto_2::getGestures::Request &reqLocal) const;
};


