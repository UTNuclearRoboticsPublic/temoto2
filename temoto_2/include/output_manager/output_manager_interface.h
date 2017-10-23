#include "core/common.h"

#include "common/temoto_id.h"
#include "base_task/task_errors.h"
#include "output_manager/output_manager_errors.h"
#include "output_manager/rviz_manager/rviz_manager_services.h"
#include "rmp/resource_manager.h"
#include "temoto_2/LoadRvizPlugin.h"
#include <sstream>
#include <fstream>

/**
 * @brief The OutputManagerInterface class
 */
class OutputManagerInterface
{
public:
  /**
   * @brief OutputManagerInterface
   */
  OutputManagerInterface() : resource_manager_("output_manager_interface", this)
  {
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
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    temoto_2::LoadRvizPlugin load_srv;
    load_srv.request.type = display_type;
    load_srv.request.topic = topic;
    load_srv.request.config = display_config;

    // Call the server
    if (!resource_manager_.template call<temoto_2::LoadRvizPlugin>(
            rviz_manager::srv_name::MANAGER, rviz_manager::srv_name::SERVER, load_srv))
    {
      throw error::ErrorStackUtil(taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK,
                                  error::Urgency::MEDIUM, prefix + " Failed to call service",
                                  ros::Time::now());
    }

    if (load_srv.response.rmp.code == 0)
    {
      plugins_.push_back(load_srv);
    }
    else
    {
      throw error::ErrorStackUtil(
          taskErr::SERVICE_REQ_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
          prefix + " Unsuccessful call to rviz_manager" + load_srv.response.rmp.message,
          ros::Time::now());
    }
  }

  /**
   * @brief stopAllocatedServices
   */
  void hideInRviz(std::string display_type, std::string topic = "", std::string display_config = "")
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    temoto_2::LoadRvizPlugin::Request req;
    req.type = display_type;
    req.topic = topic;
    req.config = display_config;

		auto cur_plugin_it = plugins_.begin();
		while (cur_plugin_it != plugins_.end())
		{
			// The == operator used in the lambda function is defined in
			// rviz manager services header
			auto found_plugin_it = std::find_if(cur_plugin_it, plugins_.end(),
					[&](const temoto_2::LoadRvizPlugin& srv_msg) -> 
					bool {return srv_msg.request == req;}
					);
			if (found_plugin_it != plugins_.end())
			{
				// do the unloading
				resource_manager_.unloadClientResource(found_plugin_it->response.rmp.resource_id);
				cur_plugin_it = found_plugin_it;
			}
			else if(cur_plugin_it == plugins_.begin())
			{
				throw error::ErrorStackUtil( taskErr::RESOURCE_UNLOAD_FAIL,
						error::Subsystem::TASK,
						error::Urgency::MEDIUM,
						prefix + " Unable to unload resource that is not loaded.",
						ros::Time::now());
			}
		}

  }


  /**
   * @brief displayConfigFromFile
   * @param config_path
   * @return
   */
  std::string displayConfigFromFile(std::string config_path)
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    // Create filestream object and configure exceptions
    std::ifstream config_file;
    config_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
    {
      // Open the file stream
      config_file.open(config_path);

      // Stream the file into a stringstream
      std::stringstream sstr;
      while (config_file >> sstr.rdbuf())
        ;

      return sstr.str();
    }
    catch (std::ifstream::failure e)
    {
      // Rethrow the exception
      throw error::ErrorStackUtil(
          outputManagerErr::FILE_OPEN_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
          prefix + " Failed to open the display config file", ros::Time::now());
    }
  }

  ~OutputManagerInterface()
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);
  }

  const std::string& getName() const
  {
    return class_name_;
  }

private:
  TemotoID::ID id_ = TemotoID::UNASSIGNED_ID;

  const std::string class_name_ = "OutputManagerInterface";

  rmp::ResourceManager<OutputManagerInterface> resource_manager_;

  std::vector<temoto_2::LoadRvizPlugin> plugins_;

  error::ErrorHandler error_handler_;
};
