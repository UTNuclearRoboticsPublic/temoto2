#include "output_manager/rviz_manager/plugin_info.h"


/*
 * PluginInfo implementations
 */

PluginInfo::PluginInfo (){}

PluginInfo::PluginInfo (std::string name, std::string type)
    :
    name_(name),
    type_(type)
{}

bool PluginInfo::setDescription (std::string description)
{
    this->description_ = description;
}

std::string PluginInfo::getName ()
{
    return this->name_;
}

std::string PluginInfo::getType ()
{
    return this-> type_;
}

std::string PluginInfo::getDescription ()
{
    return this->description_;
}


/*
 * PluginInfoHandler implementations
 */

std::vector <PluginInfo> PluginInfoHandler::findPlugins ( std::string plugin_type )
{
    // Empty PluginInfo vector
    std::vector <PluginInfo> plugin_info_return;

    for (auto& plugin : this->plugins_)
    {
        if (plugin.getType() == plugin_type)
            plugin_info_return.push_back( plugin );
    }

    return plugin_info_return;
}

bool PluginInfoHandler::findPlugin ( std::string plugin_type, PluginInfo& plugin_info)
{
    std::vector <PluginInfo> plugins = findPlugins( plugin_type );

    if ( plugins.size() > 0 )
    {
        plugin_info = plugins[0];
        return true;
    }
    else
    {
        return false;
    }
}
