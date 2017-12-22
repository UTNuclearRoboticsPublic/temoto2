#include "core/common.h"

#include "common/temoto_id.h"
#include "common/base_subsystem.h"
#include "TTP/base_task/task_errors.h"
#include "TTP/base_task/base_task.h"
#include "output_manager/output_manager_errors.h"
#include "output_manager/output_manager_services.h"
#include "robot_manager/robot_manager_services.h"
#include "rmp/resource_manager.h"
#include "temoto_2/LoadRvizPlugin.h"
#include <sstream>
#include <fstream>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace output_manager
{
/**
 * @brief The OutputManagerInterface class
 */
template <class OwnerTask>
class OutputManagerInterface : public BaseSubsystem
{
public:
  /**
   * @brief OutputManagerInterface
   */
  OutputManagerInterface()
  {
    class_name_ = __func__;
  }

  /**
   * @brief initialize
   */
  void initialize(TTP::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();

    name_ = task->getName() + "/output_manager_interface";
    resource_manager_ = std::unique_ptr<rmp::ResourceManager<OutputManagerInterface>>(
        new rmp::ResourceManager<OutputManagerInterface>(name_, this));
    //    resource_manager_->registerStatusCb(&OutputManagerInterface::statusInfoCb);
  }

  /**
   * @brief showInRviz
   * @param display_type
   * @param topic
   * @param display_config
   */
  void showInRviz(std::string display_type, std::string topic = "", std::string display_config = "")
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    temoto_2::LoadRvizPlugin load_srv;
    load_srv.request.type = display_type;
    load_srv.request.topic = topic;
    load_srv.request.config = display_config;

    // Call the server
    if (!resource_manager_->template call<temoto_2::LoadRvizPlugin>(
            srv_name::RVIZ_MANAGER, srv_name::RVIZ_SERVER, load_srv))
    {
      throw CREATE_ERROR(taskErr::SERVICE_REQ_FAIL, "Failed to call service");
    }

    if (load_srv.response.rmp.code == 0)
    {
      plugins_.push_back(load_srv);
    }
    else
    {
      throw CREATE_ERROR(taskErr::SERVICE_REQ_FAIL, "Unsuccessful call to rviz_manager");
    }
  }

  /**
   * @brief stopAllocatedServices
   */
  void hideInRviz(std::string display_type, std::string topic = "", std::string display_config = "")
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    temoto_2::LoadRvizPlugin::Request req;
    req.type = display_type;
    req.topic = topic;
    req.config = display_config;

    auto cur_plugin_it = plugins_.begin();
    while (cur_plugin_it != plugins_.end())
    {
      // The == operator used in the lambda function is defined in
      // rviz manager services header
      auto found_plugin_it = std::find_if(
          cur_plugin_it, plugins_.end(),
          [&](const temoto_2::LoadRvizPlugin& srv_msg) -> bool { return srv_msg.request == req; });
      if (found_plugin_it != plugins_.end())
      {
        // do the unloading
        resource_manager_->unloadClientResource(found_plugin_it->response.rmp.resource_id);
        cur_plugin_it = found_plugin_it;
      }
      else if (cur_plugin_it == plugins_.begin())
      {
        throw CREATE_ERROR(taskErr::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not loaded.");
      }
    }
  }

  void showRobot(const std::set<std::string>& visualization_options)
  {
    showRobot("", visualization_options);
  }

  void showRobot(const std::string& robot_name,
                 const std::set<std::string>& visualization_options)
  {

    YAML::Node info = YAML::Load(getRobotInfo(robot_name));
    
    if (visualization_options.find("robot_model") != visualization_options.end())
    {
      if (info["urdf"].IsMap())
      {
        std::string rob_desc_param = info["urdf"]["robot_description"].as<std::string>();
        TEMOTO_WARN("robot desc %s",rob_desc_param.c_str() );
        showInRviz("robot_model", rob_desc_param);
      }
      else
      {
        TEMOTO_ERROR("Robot does not have an urdf capability.");
      }
    }


  // --Robot description
//  std::string viz_conf = "{Robot Description: /" + act_rob_ns +
//                         "/robot_description, Move Group Namespace: /" + act_rob_ns +
//                         ", Planning Scene Topic: /" + act_rob_ns +
//                         "/move_group/monitored_planning_scene, Planning Request: {Planning Group: "
//                         "manipulator}, Planned Path: {Trajectory Topic: /" +
//                         act_rob_ns + "/move_group/display_planned_path}}";
  }

  std::string getRobotInfo(const std::string& robot_name)
  {
    std::string info;
    ros::ServiceClient rm_client;
    temoto_2::RobotGetVizInfo info_srvc;
    info_srvc.request.robot_name = robot_name;
    if (rm_client.call(info_srvc))
    {
      TEMOTO_DEBUG(" GET ROBOT INFO SUCESSFUL. Response:");
      TEMOTO_DEBUG_STREAM(info_srvc.response);
      info = info_srvc.response.info;
    }
    return info;
  }

  /*
   * @brief displayConfigFromFile
   * @param config_path
   * @return
   */
  std::string displayConfigFromFile(std::string config_path)
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    // Create filestream object and configure exceptions
    std::ifstream config_file;
    config_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
    {
      // Open the file stream
      config_file.open(config_path);

      // Stream the file into a stringstream
      std::stringstream sstr;
      while (config_file >> sstr.rdbuf());
      return sstr.str();
    }
    catch (std::ifstream::failure e)
    {
      // Rethrow the exception
      throw CREATE_ERROR(ErrorCode::FILE_OPEN_FAIL, "Failed to open the display config file");
    }
  }

  //  void statusInfoCb(temoto_2::ResourceStatus& srv)
  //  {
  //    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);
  //    validateInterface(prefix);
  //
  //    TEMOTO_DEBUG("%s status info was received", prefix.c_str());
  //    TEMOTO_DEBUG_STREAM(srv.request);
  //    // if any resource should fail, just unload it and try again
  //    // there is a chance that sensor manager gives us better sensor this time
  //    if (srv.request.status_code == rmp::status_codes::FAILED)
  //    {
  //      TEMOTO_WARN("Output manager interface detected a sensor failure. Unloading and "
  //                                "trying again");
  //      auto sens_it = std::find_if(allocated_sensors_.begin(), allocated_sensors_.end(),
  //                                  [&](const temoto_2::LoadSensor& sens) -> bool {
  //                                    return sens.response.rmp.resource_id ==
  //                                    srv.request.resource_id;
  //                                  });
  //      if (sens_it != allocated_sensors_.end())
  //      {
  //        TEMOTO_DEBUG("Unloading");
  //        resource_manager_->unloadClientResource(sens_it->response.rmp.resource_id);
  //        TEMOTO_DEBUG("Asking the same sensor again");
  //        if (!resource_manager_->template call<temoto_2::LoadSensor>(
  //                sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, *sens_it))
  //        {
  //          throw CREATE_ERROR(taskErr::SERVICE_REQ_FAIL, "Failed to call service");
  //        }
  //
  //        // If the request was fulfilled, then add the srv to the list of allocated sensors
  //        if (sens_it->response.rmp.code == 0)
  //        {
  //          // @TODO: send somehow topic to whoever is using this thing
  //          // or do topic remapping
  //        }
  //        else
  //        {
  //          throw CREATE_ERROR(taskErr::SERVICE_REQ_FAIL, "Unsuccessful call to sensor manager: ");
  //        }
  //      }
  //      else
  //      {
  //      }
  //    }
  //  }

  ~OutputManagerInterface()
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

    TEMOTO_DEBUG("OutputManagerInterface destroyed.");
  }

  const std::string& getName() const
  {
    return log_class_;
  }

private:
  std::string name_;

  std::string log_class_, log_subsys_, log_group_;

  TemotoID::ID id_ = TemotoID::UNASSIGNED_ID;

  std::unique_ptr<rmp::ResourceManager<OutputManagerInterface>> resource_manager_;

  std::vector<temoto_2::LoadRvizPlugin> plugins_;

  error::ErrorHandler error_handler_;

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if (!resource_manager_)
    {
      throw CREATE_ERROR(ErrorCode::NOT_INITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace
