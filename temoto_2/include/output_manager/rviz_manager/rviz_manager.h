#ifndef RVIZ_MANAGER_H
#define RVIZ_MANAGER_H

#include "core/common.h"
#include "common/request_container.h"
#include "common/temoto_id.h"
#include "temoto_2/ShowInRviz.h"
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"
#include "output_manager/output_manager_errors.h"
#include "output_manager/rviz_manager/plugin_info.h"
#include "output_manager/rviz_manager/rviz_manager_services.h"
#include "process_manager/process_manager_services.h"
#include "context_manager/human_context/human_context_services.h"
#include "rmp/resource_manager.h"

#include "temoto_2/stopAllocatedServices.h"

namespace 
{

class RvizManager
{
public:
    /**
     * @brief error_handler_
     */
    error::ErrorHandler error_handler_;

    RvizManager ();

    RvizManager (std::string path_to_default_conf);

private:

    ros::NodeHandle nh_;

    std::vector <RequestContainer<temoto_2::ShowInRviz>> active_requests_;

    temoto_id::IDManager id_manager_;

	temoto_id::ID rviz_resource_id_;

	rmp::ResourceManager<RvizManager> resource_manager_;

    ros::ServiceServer show_in_rviz_server_;

    ros::ServiceServer stop_allocated_services_server_;

    ros::ServiceClient load_plugin_client_;

    ros::ServiceClient unload_plugin_client_;

    ros::ServiceClient set_plugin_config_client_;

    ros::ServiceClient get_plugin_config_client_;

    const std::string class_name_ = "RvizManager";

    std::string path_to_default_conf_;

    PluginInfoHandler plugin_info_handler_;

    bool rviz_running_ = false;



    void runRviz();

    bool stopRviz();

    bool loadPluginRequest ( rviz_plugin_manager::PluginLoad& load_plugin_srv );

    bool unloadPluginRequest ( rviz_plugin_manager::PluginUnload& unload_plugin_srv );

    bool getPluginConfigRequest ( rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv );
	
    bool setPluginConfigRequest ( rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv );

    void loadRvizCb (temoto_2::ShowInRviz::Request &req,
                       temoto_2::ShowInRviz::Response &res);

    bool unloadCb (temoto_2::stopAllocatedServices::Request& req,
                                temoto_2::stopAllocatedServices::Response& res);

    PluginInfo findPlugin( std::string plugin_type );

    bool compareRequest (temoto_2::ShowInRviz::Request req1,
                         temoto_2::ShowInRviz::Request req2);
};

} // namespace rviz_manager

#endif
