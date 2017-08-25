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

    PluginInfo (std::string name, std::string type);

    bool setDescription (std::string description);

    std::string getName ();

    std::string getType ();

    std::string getDescription ();

private:

    /**
     * @brief Name of the plugin
     */
    std::string name_;

    /**
     * @brief Type of the plugin, e.g., "Hand tracker", "LIDAR", etc.
     */
    std::string type_;

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
