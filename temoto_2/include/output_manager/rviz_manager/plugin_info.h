#ifndef PLUGIN_INFO_H
#define PLUGIN_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>

class PluginInfo
{
public:

    /**
     * @brief plugin_info
     */
    PluginInfo ();

//    PluginInfo (std::string type, std::string class_name);

 //   PluginInfo (std::string type, std::string class_name, std::string rviz_name);

    PluginInfo (std::string type, std::string class_name, 
				std::string rviz_name = "", std::string data_type = "");

    void setDescription (std::string description);

    void setRvizName (std::string rviz_name);

    std::string getType ();

    std::string getClassName ();

    std::string getDataType ();

    std::string getRvizName ();

    std::string getDescription ();

private:

    /**
     * @brief Type of the plugin, e.g., "Hand tracker", "LIDAR", etc.
     */
    std::string type_;
	
    /**
     * @brief Name of the plugin
     */
    std::string class_name_;

    /**
     * @brief A data type of the plugin, e.g., std_msgs/Image.
     */
    std::string data_type_;

    /**
     * @brief Plugin name that appears in rviz plugin list, e.g., 'My Marker'.
     */
    std::string rviz_name_;

    /**
     * @brief A description of the plugin
     */
    std::string description_;
};


class PluginInfoHandler
{
public:

    /**
     * @brief Returns the first plugin that satisfies the "plugin type" condition
     * @param plugin_type
     * @param plugin_info
     * @return
     */
    bool findPlugin ( std::string plugin_type, PluginInfo& plugin_info);

    /**
     * @brief Returns a vector of plugins if the "plugin type" condition is satisfied
     * @param plugin_type
     * @return
     */
    std::vector <PluginInfo> findPlugins ( std::string plugin_type );

    std::vector<PluginInfo> plugins_;
};

#endif
