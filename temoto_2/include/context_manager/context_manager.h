#ifndef CONTEXT_MANAGER_H
#define CONTEXT_MANAGER_H

#include "common/base_subsystem.h"
#include "common/temoto_id.h"

#include "context_manager/context_manager_containers.h"
#include "context_manager/tracking_method.h"
#include "context_manager/context_manager_services.h"
#include "TTP/task_manager.h"

#include "rmp/resource_manager.h"
#include "rmp/config_synchronizer.h"

#include "sensor_manager/sensor_manager_services.h"
#include "algorithm_manager/algorithm_manager_services.h"

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
   * @brief loadTrackerCb
   * @param req
   * @param res
   */
  void loadTrackerCb(temoto_2::LoadTracker::Request& req, temoto_2::LoadTracker::Response& res);

  /**
   * @brief unloadTrackerCb
   * @param req
   * @param res
   */
  void unloadTrackerCb(temoto_2::LoadTracker::Request& req, temoto_2::LoadTracker::Response& res);

  /**
   * @brief findTrackers
   * @param req
   * @return
   */
  std::vector<TrackerInfo> findTrackers(temoto_2::LoadTracker::Request& req);

  /**
   * @brief loadTrackObjectCb
   * @param req
   * @param res
   */
  void loadTrackObjectCb(temoto_2::TrackObject::Request& req, temoto_2::TrackObject::Response& res);

  /**
   * @brief unloadTrackObjectCb
   * @param req
   * @param res
   */
  void unloadTrackObjectCb(temoto_2::TrackObject::Request& req, temoto_2::TrackObject::Response& res);

  /**
   * @brief parseTrackers
   * @param config_path
   */
  void parseTrackers(std::string config_path);

  /**
   * @brief Service that sets up a gesture publisher
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  void loadGestureCb(temoto_2::LoadGesture::Request& req, temoto_2::LoadGesture::Response& res);

  /**
   * @brief Unload Callback for gesture
   * @param LoadGesture request message
   * @param LoadGesture response message
   * @return
   */
  void unloadGestureCb(temoto_2::LoadGesture::Request& req, temoto_2::LoadGesture::Response& res);

  /**
   * @brief Service that sets up a speech publisher
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  void loadSpeechCb(temoto_2::LoadSpeech::Request& req, temoto_2::LoadSpeech::Response& res);

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

  void objectSyncCb(const temoto_2::ConfigSync& msg, const Objects& payload);

  void advertiseAllObjects();

  void addOrUpdateObjects(const Objects& objects_to_add, bool from_other_manager);

  ObjectPtr findObject(std::string object_name);


  // Resource manager for handling servers and clients
  rmp::ResourceManager<ContextManager> resource_manager_1_;

  /*
   * Resource manager for handling servers and clients.
   * TODO: The second manager is used for making RMP calls within the same manager. If the same
   * resouce manager is used for calling servers managed by the same manager, the calls will lock
   */
  rmp::ResourceManager<ContextManager> resource_manager_2_;

  ros::NodeHandle nh_;

  ros::ServiceServer add_objects_server_;

  ObjectPtrs objects_;

  std::map<int, std::string> m_tracked_objects_;

  std::map<std::string, std::vector<context_manager::TrackerInfo>> categorized_trackers_;

  temoto_id::IDManager pipe_id_generator_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  rmp::ConfigSynchronizer<ContextManager, Objects> object_syncer_;

  TTP::TaskManager tracker_core_;
};
}

#endif
