#ifndef CONTEXT_MANAGER_H
#define CONTEXT_MANAGER_H

#include "core/common.h"
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "context_manager/context_manager_services.h"
#include "sensor_manager/sensor_manager_services.h"
#include "rmp/resource_manager.h"
#include "rmp/config_synchronizer.h"

namespace context_manager
{
class ContextManager : public BaseSubsystem
{
public:
  ContextManager();

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:
  /**
   * @brief Service that sets up a gesture publisher
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  void loadGestureCb(temoto_2::LoadGesture::Request& req, temoto_2::LoadGesture::Response& res);

  /**
   * @brief Service that sets up a speech publisher
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  void loadSpeechCb(temoto_2::LoadSpeech::Request& req, temoto_2::LoadSpeech::Response& res);

  /**
   * @brief Unload Callback for gesture
   * @param LoadGesture request message
   * @param LoadGesture response message
   * @return
   */

  void unloadGestureCb(temoto_2::LoadGesture::Request& req, temoto_2::LoadGesture::Response& res);
  /**
   * @brief Unload Callback for speech
   * @param LoadSpeech request message
   * @param LoadSpeech response message
   * @return
   */
  void unloadSpeechCb(temoto_2::LoadSpeech::Request& req, temoto_2::LoadSpeech::Response& res);

  /**
   * @brief addObjectCb
   * @param req
   * @param res
   */
  bool addObjectCb(temoto_2::AddObject::Request& req, temoto_2::AddObject::Response& res);

  // Resource manager for handling servers and clients
  rmp::ResourceManager<ContextManager> resource_manager_;

  ros::NodeHandle nh_;

  ros::ServiceServer add_object_server_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  // rmp::ConfigSynchronizer<ContextManager> config_syncer_;
};
}

#endif
