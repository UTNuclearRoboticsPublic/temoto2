#include "output_manager/rviz_manager/plugin_info.h"


/*
 * PluginInfo implementations
 */

// TODO: disable default constructor
PluginInfo::PluginInfo (){}

/*
PluginInfo::PluginInfo (std::string type, std::string class_name)
	: PluginInfo(type, class_name, "", "")
{
}

PluginInfo::PluginInfo (std::string type, std::string class_name, std::string rviz_name)
	: PluginInfo(type, class_name, rviz_name, "")
{
}
*/

PluginInfo::PluginInfo (std::string type, std::string class_name,
	   					std::string rviz_name, std::string data_type)
    :
    type_(type),
    class_name_(class_name),
    data_type_(data_type),
    rviz_name_(rviz_name),
    description_("")
{}

void PluginInfo::setDescription (std::string description)
{
    this->description_ = description;
}

void PluginInfo::setRvizName (std::string rviz_name)
{
    this->rviz_name_ = rviz_name;
}

std::string PluginInfo::getType ()
{
    return this-> type_;
}

std::string PluginInfo::getClassName ()
{
    return this->class_name_;
}

std::string PluginInfo::getDataType ()
{
    return this->data_type_;
}

std::string PluginInfo::getRvizName ()
{
    return this->rviz_name_;
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
