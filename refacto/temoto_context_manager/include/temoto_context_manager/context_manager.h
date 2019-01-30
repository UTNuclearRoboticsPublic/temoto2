#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/reliability.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/rmp/config_synchronizer.h"

#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/tracking_method.h"
#include "temoto_context_manager/context_manager_services.h"

#include "temoto_nlp/task_manager.h"
#include "temoto_component_manager/component_manager_services.h"

namespace temoto_context_manager
{

typedef std::shared_ptr<temoto_context_manager::TrackerInfo> TrackerInfoPtr;
typedef std::vector<TrackerInfoPtr> TrackerInfoPtrs;

class ContextManager : public temoto_core::BaseSubsystem
{
public:
  ContextManager();

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  void loadGetNumberCb(GetNumber::Request& req, GetNumber::Response& res);

  void unloadGetNumberCb(GetNumber::Request& req, GetNumber::Response& res);

  /**
   * @brief loadTrackerCb
   * @param req
   * @param res
   */
  void loadTrackerCb(LoadTracker::Request& req, LoadTracker::Response& res);

  /**
   * @brief unloadTrackerCb
   * @param req
   * @param res
   */
  void unloadTrackerCb(LoadTracker::Request& req, LoadTracker::Response& res);

  /**
   * @brief findTrackers
   * @param req
   * @return
   */
  TrackerInfoPtrs findTrackers(LoadTracker::Request& req);

  /**
   * @brief loadTrackObjectCb
   * @param req
   * @param res
   */
  void loadTrackObjectCb(TrackObject::Request& req, TrackObject::Response& res);

  /**
   * @brief unloadTrackObjectCb
   * @param req
   * @param res
   */
  void unloadTrackObjectCb(TrackObject::Request& req, TrackObject::Response& res);

  /**
   * @brief parseTrackers
   * @param config_path
   */
  void parseTrackers(std::string config_path);

  /**
   * @brief Service that sets up a speech publisher
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  void loadSpeechCb(LoadSpeech::Request& req, LoadSpeech::Response& res);

  /**
   * @brief Unload Callback for speech
   * @param LoadSpeech request message
   * @param LoadSpeech response message
   * @return
   */
  void unloadSpeechCb(LoadSpeech::Request& req, LoadSpeech::Response& res);

  /**
   * @brief addObjectCb
   * @param req
   * @param res
   */
  bool addObjectsCb(AddObjects::Request& req, AddObjects::Response& res);

  void objectSyncCb(const temoto_core::ConfigSync& msg, const Objects& payload);

  void trackedObjectsSyncCb(const temoto_core::ConfigSync& msg, const std::string& payload);

  void advertiseAllObjects();

  void addOrUpdateObjects(const Objects& objects_to_add, bool from_other_manager);

  ObjectPtr findObject(std::string object_name);

  void statusCb1(temoto_core::ResourceStatus& srv);

  void statusCb2(temoto_core::ResourceStatus& srv);

  void addDetectionMethod(std::string detection_method);

  void addDetectionMethods(std::vector<std::string> detection_methods);

  std::vector<std::string> getOrderedDetectionMethods();



  // Resource manager for handling servers and clients
  temoto_core::rmp::ResourceManager<ContextManager> resource_manager_1_;

  /*
   * Resource manager for handling servers and clients.
   * TODO: The second manager is used for making RMP calls within the same manager. If the same
   * resouce manager is used for calling servers managed by the same manager, the calls will lock
   */
  temoto_core::rmp::ResourceManager<ContextManager> resource_manager_2_;

  ros::NodeHandle nh_;

  ros::ServiceServer add_objects_server_;

  ObjectPtrs objects_;

  std::map<int, std::string> m_tracked_objects_local_;

  std::map<std::string, std::string> m_tracked_objects_remote_;

  std::map<std::string, TrackerInfoPtrs> categorized_trackers_;

  std::map<int, TrackerInfoPtr> allocated_trackers_;

  temoto_core::temoto_id::IDManager pipe_id_generator_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  temoto_core::rmp::ConfigSynchronizer<ContextManager, Objects> object_syncer_;

  temoto_core::rmp::ConfigSynchronizer<ContextManager, std::string> tracked_objects_syncer_;

  temoto_nlp::TaskManager tracker_core_;

  std::map<std::string, temoto_core::Reliability> detection_method_history_;

  std::pair<int, std::string> active_detection_method_;

  /*
   * TODO: A DATA STRUCTURE THAT IS A TEMPORARY HACK UNTIL RMP IS IMPROVED
   */
  std::map<int, std::pair<TrackerInfoPtr, std::vector<int>>>  allocated_trackers_hack_;
};
} // temoto_context_manager namespace

#endif
