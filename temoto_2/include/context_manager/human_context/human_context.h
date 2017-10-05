#include "core/common.h"
#include "common/temoto_id.h"
#include "rmp/resource_manager.h"
#include "context_manager/human_context/human_context_services.h"

class HumanContext
{
public:

    HumanContext();

private:
    ros::NodeHandle nh_;

    // Resource manager for handling servers and clients
	rmp::ResourceManager<HumanContext> resource_manager_;

    /**
     * @brief Service that sets up a gesture publisher
     * @param A gesture specifier message
     * @param Returns a topic where the requested gesture messages
     * are going to be published
     * @return
     */
    void loadGestureCb (temoto_2::LoadGestures::Request &req,
                        temoto_2::LoadGestures::Response &res);

    /**
     * @brief Service that sets up a speech publisher
     * @param A gesture specifier message
     * @param Returns a topic where the requested gesture messages
     * are going to be published
     * @return
     */
    void loadSpeechCb (temoto_2::LoadSpeech::Request &req,
                       temoto_2::LoadSpeech::Response &res);
};


