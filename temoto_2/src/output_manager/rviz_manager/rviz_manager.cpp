#include "output_manager/rviz_manager/rviz_manager.h"

RvizManager::RvizManager()
{
    show_in_rviz_server_ = n_.advertiseService("show_in_rviz", &RvizManager::showInRvizCb, this);
    node_spawn_kill_client_ = n_.serviceClient<temoto_2::nodeSpawnKill>("spawn_kill_process");
    load_plugin_client_ = n_.serviceClient<temoto_2::loadPlugin>("load_rviz_plugin");
}


RvizManager::RvizManager(std::string path_to_default_conf)
    :
    path_to_default_conf_(path_to_default_conf)
{
    show_in_rviz_server_ = n_.advertiseService("show_in_rviz", &RvizManager::showInRvizCb, this);
    node_spawn_kill_client_ = n_.serviceClient<temoto_2::nodeSpawnKill>("spawn_kill_process");
    load_plugin_client_ = n_.serviceClient<temoto_2::loadPlugin>("load_rviz_plugin");
}

bool RvizManager::runRviz()
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "runRviz";
    std::string prefix = formatMessage("", this->class_name_, method_name_);

    // Create the message and fill out the request part
    temoto_2::nodeSpawnKill spawn_kill_msg;
    spawn_kill_msg.request.action = "roslaunch";
    spawn_kill_msg.request.package = "temoto_2";
    spawn_kill_msg.request.name = "rviz.launch";

    ROS_INFO("%s Requesting to launch rviz ...", prefix.c_str());

    // Call the server
    if (node_spawn_kill_client_.call(spawn_kill_msg))
    {
        if (spawn_kill_msg.response.code == 0)
        {
            ROS_INFO("%s Rviz launched succesfully: %s", prefix.c_str(), spawn_kill_msg.response.message.c_str());
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::RVIZ_OPEN_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to launch rviz: " + spawn_kill_msg.response.message,
                                         ros::Time::now());
        }
    }
    else
    {
        throw error::ErrorStackUtil( outputManagerErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::OUTPUT_MANAGER,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service /spawn_kill_process",
                                     ros::Time::now());
    }
}

bool RvizManager::sendPluginRequest ( temoto_2::loadPlugin& load_plugin_srv )
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "sendPluginRequest";
    std::string prefix = formatMessage("", this->class_name_, method_name_);

    // Send the plugin request
    if ( load_plugin_client_.call(load_plugin_srv) )
    {
        if ( load_plugin_srv.response.code == 0 )
        {
            ROS_INFO("%s Request successful: %s", prefix.c_str(), load_plugin_srv.response.message.c_str());
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::PLUGIN_LOAD_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to load rviz plugin: " + load_plugin_srv.response.message,
                                         ros::Time::now());
        }
    }
    else
    {
        throw error::ErrorStackUtil( outputManagerErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::OUTPUT_MANAGER,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service /load_rviz_plugin",
                                     ros::Time::now());
    }
}


bool RvizManager::showInRvizCb( temoto_2::showInRviz::Request &req,
                                temoto_2::showInRviz::Response &res )
{
    // Name of the method, used for making debugging a bit simpler
    const std::string method_name_ = "showInRvizCb";
    std::string prefix = formatMessage("", this->class_name_, method_name_);

    ROS_DEBUG("%s Received a 'show_in_rviz' request", prefix.c_str());

    // If Rviz is not running yet, then send a launch request to the Node Manager
    if (!rviz_running_)
    {
        try
        {
            runRviz();
            rviz_running_ = true;
        }
        catch( error::ErrorStackUtil& e )
        {
            // Format the service response
            res.code = 1;
            res.message = e.getStack().back().message;

            // Append the error to local ErrorStack
            e.forward( prefix );
            this->error_handler_.append(e);

            return true;
        }
    }

    // Check the type of the requested display plugin and run if found
    PluginInfo plugin_info;

    if ( plugin_info_handler_.findPlugin( req.type, plugin_info ) )
    {
        // Create the message and fill out the request part
        temoto_2::loadPlugin load_plugin_srv;
        load_plugin_srv.request.plugin_name = plugin_info.getName();
        load_plugin_srv.request.topic = req.topic;

        try
        {
            sendPluginRequest (load_plugin_srv);
            res.code = load_plugin_srv.response.code;
            res.message = load_plugin_srv.response.message;

            return true;
        }
        catch( error::ErrorStackUtil & e )
        {
            // Format the service response
            res.code = 1;
            res.message = e.getStack().back().message;

            // Append the error to local ErrorStack
            e.forward( prefix );
            this->error_handler_.append(e);

            return true;
        }
    }
}
