#include "output_manager/rviz_manager/rviz_manager.h"

RvizManager::RvizManager()
{
    show_in_rviz_server_ = n_.advertiseService("show_in_rviz", &RvizManager::showInRvizCb, this);
    stop_allocated_services_server_ = n_.advertiseService( "stop_allocated_services_om",
                                                           &RvizManager::stopAllocatedServices,
                                                           this);

    node_spawn_kill_client_ = n_.serviceClient<temoto_2::nodeSpawnKill>("spawn_kill_process");
    load_plugin_client_ = n_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
    unload_plugin_client_ = n_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
    get_plugin_config_client_ = n_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_config");
    set_plugin_config_client_ = n_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_config");

    /*
     * Add some plugin entries to the "plugin_info_handler_". This should be done via
     * external xml file or a service request
     */
    plugin_info_handler_.plugins_.emplace_back( "rviz/Marker", "marker" );
    plugin_info_handler_.plugins_.emplace_back( "rviz_textured_sphere/SphereDisplay", "camera" );
    plugin_info_handler_.plugins_.emplace_back( "rviz/Image", "image" );
}

/*
RvizManager::RvizManager(std::string path_to_default_conf)
    :
    path_to_default_conf_(path_to_default_conf)
{
    show_in_rviz_server_ = n_.advertiseService("show_in_rviz", &RvizManager::showInRvizCb, this);
    node_spawn_kill_client_ = n_.serviceClient<temoto_2::nodeSpawnKill>("spawn_kill_process");
    load_plugin_client_ = n_.serviceClient<temoto_2::loadPlugin>("load_rviz_plugin");
}
*/

/* * * * * * * * * * * * * * * * *
 *  runRviz
 * * * * * * * * * * * * * * * * */

bool RvizManager::runRviz()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Create the message and fill out the request part
    temoto_2::nodeSpawnKill spawn_kill_msg;
    spawn_kill_msg.request.action = "roslaunch";
    spawn_kill_msg.request.package = "rviz_plugin_manager";
    spawn_kill_msg.request.name = "rviz_plugin_manager.launch";

    ROS_INFO("%s Requesting to launch rviz ...", prefix.c_str());

    // Call the server
    if( node_spawn_kill_client_.call(spawn_kill_msg))
    {
        if( spawn_kill_msg.response.code == 0)
        {
            ROS_INFO("%s Rviz launched succesfully: %s", prefix.c_str(), spawn_kill_msg.response.message.c_str());
            rviz_running_ = true;

            // Give some time for the rviz_plugin_manager to setup
            ros::Duration(3).sleep();

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

/* * * * * * * * * * * * * * * * *
 *  stopRviz
 * * * * * * * * * * * * * * * * */

bool RvizManager::stopRviz()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Create the message and fill out the request part
    temoto_2::nodeSpawnKill spawn_kill_msg;
    spawn_kill_msg.request.action = "kill";
    spawn_kill_msg.request.package = "rviz_plugin_manager";
    spawn_kill_msg.request.name = "rviz_plugin_manager.launch";

    ROS_INFO("%s Requesting to stop rviz ...", prefix.c_str());

    // Call the server
    if( node_spawn_kill_client_.call(spawn_kill_msg))
    {
        if( spawn_kill_msg.response.code == 0)
        {
            ROS_INFO("%s Rviz stopped succesfully: %s", prefix.c_str(), spawn_kill_msg.response.message.c_str());
            rviz_running_ = false;
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::RVIZ_OPEN_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to stop rviz: " + spawn_kill_msg.response.message,
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

/* * * * * * * * * * * * * * * * *
 *  loadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::loadPluginRequest( rviz_plugin_manager::PluginLoad& load_plugin_srv )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Send the plugin request
    if( load_plugin_client_.call(load_plugin_srv) )
    {
        if( load_plugin_srv.response.code == 0 )
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
                                     prefix + " Failed to call service /rviz_plugin_load",
                                     ros::Time::now());
    }
}

/* * * * * * * * * * * * * * * * *
 *  unloadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::unloadPluginRequest( rviz_plugin_manager::PluginUnload& unload_plugin_srv )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Send the plugin request
    if( unload_plugin_client_.call(unload_plugin_srv) )
    {
        if( unload_plugin_srv.response.code == 0 )
        {
            ROS_INFO("%s Request successful: %s", prefix.c_str(), unload_plugin_srv.response.message.c_str());
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::PLUGIN_UNLOAD_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to unload rviz plugin: " + unload_plugin_srv.response.message,
                                         ros::Time::now());
        }
    }
    else
    {
        throw error::ErrorStackUtil( outputManagerErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::OUTPUT_MANAGER,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service /rviz_plugin_unload",
                                     ros::Time::now());
    }
}

/* * * * * * * * * * * * * * * * *
 *  getPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::getPluginConfigRequest( rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Send the plugin request
    if( get_plugin_config_client_.call(get_plugin_config_srv) )
    {
        if( get_plugin_config_srv.response.code == 0 )
        {
            ROS_INFO("%s Request successful: %s", prefix.c_str(), get_plugin_config_srv.response.message.c_str());
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::PLUGIN_GET_CONFIG_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to get rviz plugin config: " + get_plugin_config_srv.response.message,
                                         ros::Time::now());
        }
    }
    else
    {
        throw error::ErrorStackUtil( outputManagerErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::OUTPUT_MANAGER,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service /rviz_plugin_get_config",
                                     ros::Time::now());
    }
}


/* * * * * * * * * * * * * * * * *
 *  setPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::setPluginConfigRequest( rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Send the plugin request
    if( set_plugin_config_client_.call(set_plugin_config_srv) )
    {
        if( set_plugin_config_srv.response.code == 0 )
        {
            ROS_INFO("%s Request successful: %s", prefix.c_str(), set_plugin_config_srv.response.message.c_str());
            return true;
        }
        else
        {
            throw error::ErrorStackUtil( outputManagerErr::PLUGIN_SET_CONFIG_FAIL,
                                         error::Subsystem::OUTPUT_MANAGER,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to set rviz plugin config: " + set_plugin_config_srv.response.message,
                                         ros::Time::now());
        }
    }
    else
    {
        throw error::ErrorStackUtil( outputManagerErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::OUTPUT_MANAGER,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service /rviz_plugin_set_config",
                                     ros::Time::now());
    }
}

/* * * * * * * * * * * * * * * * *
 *  showInRvizCb
 * * * * * * * * * * * * * * * * */

bool RvizManager::showInRvizCb( temoto_2::ShowInRviz::Request &req,
                                temoto_2::ShowInRviz::Response &res )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    ROS_INFO("%s Received a 'show_in_rviz' request", prefix.c_str());

    // Check the id of the req. If there is none (the first call from a task) then provide one
    TemotoID id_local = id_manager_.checkID(req.id);

    for( auto& active_req : active_requests_ )
    {
        if( compareRequest(req, active_req.getReq()) )
        {
            ROS_INFO("%s Same request already available.", prefix.c_str());
            res = active_req.getRes();
            res.id = static_cast<int>(id_local);

            // Add the ID of the client to the active_req ID list
            active_req.addID(id_local);
            return true;
        }
    }

    // Check the type of the requested display plugin and run if found
    PluginInfo plugin_info;

    if( plugin_info_handler_.findPlugin( req.type, plugin_info ) )
    {
        // If Rviz is not running yet, then send a launch request to the Node Manager
        if( !rviz_running_ )
        {
            try
            {
                runRviz();
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

        // Create the message and fill out the request part
        rviz_plugin_manager::PluginLoad load_plugin_srv;
        load_plugin_srv.request.plugin_class = plugin_info.getName();
        load_plugin_srv.request.plugin_topic = req.topic;
        load_plugin_srv.request.plugin_name = "test_test_test";

        try
        {
            loadPluginRequest (load_plugin_srv);

            // Fill out the response message
            res.id = static_cast<int>(id_local);
            res.plugin_uid = load_plugin_srv.response.plugin_uid;
            res.code = load_plugin_srv.response.code;
            res.message = load_plugin_srv.response.message;

            // Add the request and ID into the active_requests_
            temoto_2::ShowInRviz show_in_rviz_local;
            show_in_rviz_local.request = req;
            show_in_rviz_local.response = res;
            active_requests_.emplace_back (show_in_rviz_local, id_local);

			if (req.config != "")
			{
				// Send the request to set the plugin config
				rviz_plugin_manager::PluginSetConfig set_plugin_config_srv;
                                set_plugin_config_srv.request.plugin_uid = load_plugin_srv.response.plugin_uid;
                                set_plugin_config_srv.request.config = req.config;
				setPluginConfigRequest(set_plugin_config_srv);
			}

            return true;
        }
        catch( error::ErrorStackUtil &e )
        {
            // Format the service response
            res.code = 1;
            res.message = e.getStack().back().message;

            // Append the error to local ErrorStack
            e.forward( prefix );
            this->error_handler_.append(e);
        }

        // If something went wrong and there are no active resource requests
        // then close rviz
        if( rviz_running_ && active_requests_.empty() )
        {
            try
            {
                stopRviz();
            }
            catch( error::ErrorStackUtil &e )
            {
                // Append the error to local ErrorStack
                e.forward( prefix );
                this->error_handler_.append(e);
                return true;
            }
        }
    }
    else
    {
        error::ErrorStackUtil e( outputManagerErr::SERVICE_REQ_FAIL,
                                 error::Subsystem::OUTPUT_MANAGER,
                                 error::Urgency::MEDIUM,
                                 prefix + " Did not find any appropriate display plugins",
                                 ros::Time::now());

        res.code = 1;
        res.message = e.getStack().back().message;

        this->error_handler_.append(e);

        return true;
    }
}

/* * * * * * * * * * * * * * * * *
 *  stopAllocatedServices
 * * * * * * * * * * * * * * * * */

bool RvizManager::stopAllocatedServices( temoto_2::stopAllocatedServices::Request& req,
                                         temoto_2::stopAllocatedServices::Response& res)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    ROS_INFO("%s Received a 'stopAllocatedServices' request to ID: '%d'.", prefix.c_str(), static_cast<int>(req.id));

    // Default the response code to 0
    res.code = 0;

    bool id_match_found = false;
    bool active_request_erased = false;

    // Go through the  and "active_requests_" list, look for the id
    for( auto itr = active_requests_.begin(); itr != active_requests_.end(); /*empty*/ )
    {
        // Loop over IDs
        for(auto &id : (*itr).IDs_)
        {
            // Check if the id matches. Remove it if it does
            if( id == req.id)
            {
                id_match_found = true;
                (*itr).removeID( id );

                // Check if there are any clients using the same resource.
                // If not, then send a plugin unload request
                if( (*itr).IDs_.empty() )
                {
                    rviz_plugin_manager::PluginUnload plugin_unload_srv;
                    plugin_unload_srv.request.plugin_uid = (*itr).request_.response.plugin_uid;

                    // Call the service
                    try
                    {
                        unloadPluginRequest( plugin_unload_srv );
                        active_requests_.erase( itr );
                        active_request_erased = true;
                    }
                    catch( error::ErrorStackUtil &e )
                    {
                        // Append the error to local ErrorStack
                        e.forward( prefix );
                        this->error_handler_.append(e);

                        res.code = 1;
                        res.message += e.getStack().back().message + ";\n";
                    }
                }

                break;
            }
        }

        if( active_request_erased )
        {
            active_request_erased = false;

            // If there are no active requests, then close rviz
            if( active_requests_.empty() )
            {
                try
                {
                    stopRviz();
                }
                catch( error::ErrorStackUtil &e )
                {
                    // Append the error to local ErrorStack
                    e.forward( prefix );
                    this->error_handler_.append(e);
                }
            }
        }
        else
        {
            itr++;
        }
    }

    // After all this crazyness, check if the ID was found or not
    if( id_match_found )
    {
        res.code = 0;
        res.message = "Request satisfied";
    }
    else
    {
        res.code = 1;
        res.message = "No allocated resources were found";
    }

    return true;
}


/* * * * * * * * * * * * * * * * *
 *  Compare "ShowInRviz" request
 *  TODO: * Implement a complete comparison
 * * * * * * * * * * * * * * * * */

bool RvizManager::compareRequest (temoto_2::ShowInRviz::Request req1,
                                  temoto_2::ShowInRviz::Request req2)
{
    if (req1.type == req2.type)
        return true;

    return false;
}
