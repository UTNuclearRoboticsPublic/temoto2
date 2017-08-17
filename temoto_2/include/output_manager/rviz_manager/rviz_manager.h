#ifndef RVIZ_MANAGER_H
#define RVIZ_MANAGER_H

#include "core/common.h"
#include "temoto_2/nodeSpawnKill.h"
#include "temoto_2/showInRviz.h"
#include "temoto_2/loadPlugin.h"
#include "output_manager/output_manager_errors.h"
#include "output_manager/rviz_manager/plugin_info.h"

class RvizManager
{
public:
    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    RvizManager(std::string path_to_default_conf);

private:

    const std::string class_name_ = "RvizManager";

    std::string path_to_default_conf_;

    PluginInfoHandler plugin_info_handler_;

    bool rviz_running = false;


    ros::ServiceServer show_in_rviz_server_;

    ros::ServiceClient load_plugin_client_;

    ros::ServiceClient node_spawn_kill_client_;

    bool runRviz();

    bool sendPluginRequest ( temoto_2::loadPlugin& load_plugin_srv );

    bool showInRvizCb( temoto_2::showInRviz::Request &req,
                       temoto_2::showInRviz::Response &res );

    PluginInfo findPlugin( std::string plugin_type );
};

#endif
