#ifndef CONTEXT_MANAGER_H
#define CONTEXT_MANAGER_H

#include "core/common.h"
#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "context_manager_containers.h"
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

  void objectSyncCb(const temoto_2::ConfigSync& msg, const Objects& payload);

  void advertiseAllObjects();

  void addOrUpdateObjects(const Objects& objects_to_add, bool from_other_manager);

  //void loadTrackerCb(temoto_2::LoadGesture::Request& req, temoto_2::LoadGesture::Response& res);

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
  bool addObjectsCb(temoto_2::AddObjects::Request& req, temoto_2::AddObjects::Response& res);

  // Resource manager for handling servers and clients
  rmp::ResourceManager<ContextManager> resource_manager_1_;

  // Resource manager for handling servers and clients.
  // TODO: The second manager is used for making RMP calls within the same manager. If the same
  // resouce manager is used for calling servers managed by the same manager, the calls will lock
  rmp::ResourceManager<ContextManager> resource_manager_2_;

  ros::NodeHandle nh_;

  ros::ServiceServer add_objects_server_;

  ObjectPtrs objects_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  rmp::ConfigSynchronizer<ContextManager, Objects> object_syncer_;
};
}

#endif
