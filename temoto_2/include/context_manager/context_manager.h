#ifndef CONTEXT_MANAGER_H
#define CONTEXT_MANAGER_H

#include "core/common.h"
#include "common/temoto_id.h"
#include "context_manager/context_manager_services.h"
#include "sensor_manager/sensor_manager_services.h"
#include "rmp/resource_manager.h"
#include "rmp/config_synchronizer.h"

namespace context_manager
{
class ContextManager
{
public:
  ContextManager();

  const std::string& getName() const
  {
    return log_subsys_;
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

  std::string log_class_, log_subsys_, log_group_;

  // Resource manager for handling servers and clients
  rmp::ResourceManager<ContextManager> resource_manager_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  // rmp::ConfigSynchronizer<ContextManager> config_syncer_;
};
}

#endif
