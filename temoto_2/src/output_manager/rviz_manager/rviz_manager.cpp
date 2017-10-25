#include "output_manager/rviz_manager/rviz_manager.h"

namespace rviz_manager
{
RvizManager::RvizManager() : resource_manager_(srv_name::MANAGER, this)
{

  log_class_ = "";
  log_subsys_ = "rviz_manager";
  log_group_ = "rviz_manager";
  // Set up server for loading rviz plugins
  resource_manager_.addServer<temoto_2::LoadRvizPlugin>(rviz_manager::srv_name::SERVER,
                                                        &RvizManager::LoadRvizPluginCb,
                                                        &RvizManager::unloadRvizPluginCb);

  // Load rviz_plugin_manager service clients
  load_plugin_client_ = nh_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
  unload_plugin_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
  get_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_"
                                                              "config");
  set_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_"
                                                              "config");

  /*
   * Add some plugin entries to the "plugin_info_handler_". This should be done
   * via
   * external xml file or a service request
   */
  // plugin_info_handler_.plugins_.emplace_back( "marker", "rviz/Marker",
  // "Temoto Marker" );
  plugin_info_handler_.plugins_.emplace_back("marker", "rviz/Marker");
  plugin_info_handler_.plugins_.emplace_back("camera", "rviz_textured_sphere/SphereDisplay",
                                             "Temoto camera");
  plugin_info_handler_.plugins_.emplace_back("image", "rviz/Image", "Temoto Image", "sensor_msgs/"
                                                                                    "Image");
}

/* * * * * * * * * * * * * * * * *
 *  runRviz
 * * * * * * * * * * * * * * * * */
void RvizManager::runRviz()
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  // Create the message and fill out the request part
  temoto_2::LoadProcess msg;
  msg.request.action = "roslaunch";
  msg.request.package_name = "rviz_plugin_manager";
  msg.request.executable = "rviz_plugin_manager.launch";

  TEMOTO_INFO("%s Requesting to launch rviz ...", prefix.c_str());

  try
  {
    // Ask process manager to start rviz
    resource_manager_.call<temoto_2::LoadProcess>(process_manager::srv_name::MANAGER,
                                                  process_manager::srv_name::SERVER, msg);
  }
  catch (...)
  {
    throw error::ErrorStackUtil(outputManagerErr::SERVICE_REQ_FAIL,
                                error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
                                prefix + " Failed to start RViz", ros::Time::now());
  }

  if (msg.response.rmp.code == 0)
  {
    TEMOTO_INFO("%s Rviz launched succesfully: %s", prefix.c_str(), msg.response.rmp.message.c_str());

    // Give some time for the rviz_plugin_manager to setup
    ros::Duration(3).sleep();  // TODO: THIS THING SEEMS SO WRONG HERE
  }
  else
  {
    throw error::ErrorStackUtil(
        outputManagerErr::RVIZ_OPEN_FAIL, error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
        prefix + " Failed to launch rviz: " + msg.response.rmp.message, ros::Time::now());
  }
}

/* * * * * * * * * * * * * * * * *
 *  loadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::loadPluginRequest(rviz_plugin_manager::PluginLoad& load_plugin_srv)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  // Send the plugin request
  if (load_plugin_client_.call(load_plugin_srv))
  {
    if (load_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO("%s Request successful: %s", prefix.c_str(),
               load_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw error::ErrorStackUtil(
          outputManagerErr::PLUGIN_LOAD_FAIL, error::Subsystem::OUTPUT_MANAGER,
          error::Urgency::MEDIUM,
          prefix + " Failed to load rviz plugin: " + load_plugin_srv.response.message,
          ros::Time::now());
    }
  }
  else
  {
    throw error::ErrorStackUtil(outputManagerErr::SERVICE_REQ_FAIL,
                                error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
                                prefix + " Failed to call service /rviz_plugin_load",
                                ros::Time::now());
  }
}

/* * * * * * * * * * * * * * * * *
 *  unloadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::unloadPluginRequest(rviz_plugin_manager::PluginUnload& unload_plugin_srv)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  // Send the plugin request
  if (unload_plugin_client_.call(unload_plugin_srv))
  {
    if (unload_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO("%s Request successful: %s", prefix.c_str(),
               unload_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw error::ErrorStackUtil(
          outputManagerErr::PLUGIN_UNLOAD_FAIL, error::Subsystem::OUTPUT_MANAGER,
          error::Urgency::MEDIUM,
          prefix + " Failed to unload rviz plugin: " + unload_plugin_srv.response.message,
          ros::Time::now());
    }
  }
  else
  {
    throw error::ErrorStackUtil(outputManagerErr::SERVICE_REQ_FAIL,
                                error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
                                prefix + " Failed to call service /rviz_plugin_unload",
                                ros::Time::now());
  }
}

/* * * * * * * * * * * * * * * * *
 *  getPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::getPluginConfigRequest(
    rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  // Send the plugin request
  if (get_plugin_config_client_.call(get_plugin_config_srv))
  {
    if (get_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO("%s Request successful: %s", prefix.c_str(),
               get_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw error::ErrorStackUtil(
          outputManagerErr::PLUGIN_GET_CONFIG_FAIL, error::Subsystem::OUTPUT_MANAGER,
          error::Urgency::MEDIUM,
          prefix + " Failed to get rviz plugin config: " + get_plugin_config_srv.response.message,
          ros::Time::now());
    }
  }
  else
  {
    throw error::ErrorStackUtil(outputManagerErr::SERVICE_REQ_FAIL,
                                error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
                                prefix + " Failed to call service /rviz_plugin_get_config",
                                ros::Time::now());
  }
}

/* * * * * * * * * * * * * * * * *
 *  setPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::setPluginConfigRequest(
    rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  // Send the plugin request
  if (set_plugin_config_client_.call(set_plugin_config_srv))
  {
    if (set_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO("%s Request successful: %s", prefix.c_str(),
               set_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw error::ErrorStackUtil(
          outputManagerErr::PLUGIN_SET_CONFIG_FAIL, error::Subsystem::OUTPUT_MANAGER,
          error::Urgency::MEDIUM,
          prefix + " Failed to set rviz plugin config: " + set_plugin_config_srv.response.message,
          ros::Time::now());
    }
  }
  else
  {
    throw error::ErrorStackUtil(outputManagerErr::SERVICE_REQ_FAIL,
                                error::Subsystem::OUTPUT_MANAGER, error::Urgency::MEDIUM,
                                prefix + " Failed to call service /rviz_plugin_set_config",
                                ros::Time::now());
  }
}

/* * * * * * * * * * * * * * * * *
 *  loadPluginCb
 * * * * * * * * * * * * * * * * */

void RvizManager::LoadRvizPluginCb(temoto_2::LoadRvizPlugin::Request& req,
                                   temoto_2::LoadRvizPlugin::Response& res)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  TEMOTO_INFO("%s Received a new %s request", prefix.c_str(), srv_name::SERVER.c_str());
  TEMOTO_INFO_STREAM(req);

  try
  {
    runRviz();
  }
  catch (error::ErrorStackUtil& e)
  {
    // Format the rmp service response
    res.rmp.code = 1;
    res.rmp.message = e.getStack().back().message;

    // Append the error to local ErrorStack
    e.forward(prefix);
    this->error_handler_.append(e);
    return;
  }

  // Check the type of the requested display plugin and run if found
  PluginInfo plugin_info;

  // Create the message and fill out the request part
  if (plugin_info_handler_.findPlugin(req.type, plugin_info))
  {
    rviz_plugin_manager::PluginLoad load_plugin_srv;
    load_plugin_srv.request.plugin_class = plugin_info.getClassName();
    load_plugin_srv.request.plugin_topic = req.topic;
    load_plugin_srv.request.plugin_data_type = plugin_info.getDataType();
    load_plugin_srv.request.plugin_name = plugin_info.getRvizName();

    try
    {
      loadPluginRequest(load_plugin_srv);

      // Fill out the response message
      // res.plugin_uid = load_plugin_srv.response.plugin_uid;
      // store plugin uid
      res.rmp.code = load_plugin_srv.response.code;
      res.rmp.message = load_plugin_srv.response.message;

      // Add the request and ID into the active_requests_
      active_requests_.emplace(res.rmp.resource_id, load_plugin_srv.response.plugin_uid);

      // In presence of config, send it to rviz_plugin_manager
      if (req.config != "")
      {
        rviz_plugin_manager::PluginSetConfig set_plugin_config_srv;
        set_plugin_config_srv.request.plugin_uid = load_plugin_srv.response.plugin_uid;
        set_plugin_config_srv.request.config = req.config;
        setPluginConfigRequest(set_plugin_config_srv);
      }
    }
    catch (error::ErrorStackUtil& e)
    {
      // Format the service response
      res.rmp.code = 1;
      res.rmp.message = e.getStack().back().message;

      // Append the error to local ErrorStack
      e.forward(prefix);
      this->error_handler_.append(e);
    }
  }
  else
  {
    error::ErrorStackUtil e(outputManagerErr::SERVICE_REQ_FAIL, error::Subsystem::OUTPUT_MANAGER,
                            error::Urgency::MEDIUM,
                            prefix + " Did not find any appropriate display plugins",
                            ros::Time::now());
    res.rmp.code = 1;
    res.rmp.message = e.getStack().back().message;
    this->error_handler_.append(e);
    return;
  }
}

/* * * * * * * * * * * * * * * * *
 *  unloadRvizPluginCb
 * * * * * * * * * * * * * * * * */

void RvizManager::unloadRvizPluginCb(temoto_2::LoadRvizPlugin::Request& req,
                                     temoto_2::LoadRvizPlugin::Response& res)
{
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, __func__);

  TEMOTO_INFO("%s Received an 'unload' request to ID: '%ld'.", prefix.c_str(), res.rmp.resource_id);

  // Default the response code to 0
  res.rmp.code = 0;

  // Go through the  and "active_requests_" list, look for the plugin_uid

  auto it = active_requests_.find(res.rmp.resource_id);
  if (it != active_requests_.end())
  {
    rviz_plugin_manager::PluginUnload plugin_unload_srv;
    plugin_unload_srv.request.plugin_uid = it->second;

    // Call the service
    try
    {
      unloadPluginRequest(plugin_unload_srv);
      active_requests_.erase(it);
      res.rmp.code = 0;
      res.rmp.message = "Request satisfied";
    }
    catch (error::ErrorStackUtil& e)
    {
      // Append the error to local ErrorStack
      e.forward(prefix);
      this->error_handler_.append(e);

      res.rmp.code = 1;
      res.rmp.message += e.getStack().back().message + ";\n";
    }
  }
  else
  {
    res.rmp.code = 1;
    res.rmp.message = "No allocated resources were found";
  }
}

}  // namespace rviz_manager
