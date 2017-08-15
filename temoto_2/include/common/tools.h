#ifndef TOOLSX_H
#define TOOLSX_H

#include <string>

/**
 * @brief Used for createing a prefix that indicates where the message is coming from
 * @param class_name
 * @param method_name
 * @return
 */
inline std::string formatMessage (std::string subsys_name, std::string class_name, std::string method_name)
{

    if (subsys_name.compare("") == 0)
        return ("[" + class_name + "::" + method_name + "]");

    else if (class_name.compare("") == 0)
        return ("[" + subsys_name + "/" + method_name + "]");

    else
        return ("[" + subsys_name + "/" + class_name + "::" + method_name + "]");
}

#endif
