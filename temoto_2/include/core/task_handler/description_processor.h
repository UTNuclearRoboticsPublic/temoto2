#ifndef DESCRIPTION_PROCESSOR_H
#define DESCRIPTION_PROCESSOR_H

#include "core/common.h"
#include "core/task_handler/task_info.h"
#include <tinyxml.h>            // http://www.dinomage.com/2012/01/tutorial-using-tinyxml-part-1/
#include <string>
#include <vector>

class DescriptionProcessor
{

public:

    DescriptionProcessor( std::string path );

    std::string getTaskName();

    TaskInfo getTaskInfo();

    int checkTaskSignature();

    ~DescriptionProcessor();

private:

    // Params

    std::string base_path_;

    std::string desc_file_path_;

    TiXmlDocument desc_file_;

    TiXmlElement* root_element_;

    // Methods

    void openTaskDesc();

    void getRootElement();

    std::string getTaskPath();

    std::string getPackageName();

    ParamList getArgs( std::string direction );

    ParamList getInputArgs();

    ParamList getOutputArgs();

    std::vector<std::string> parseString (std::string in_str, char delimiter);

    /* TODO:
     *
     * TASK VALIDATOR METHOD - checks that there are no colliding
     *      input or output args, also calculates the checksum
     *      for making sure that the integrity of the task has
     *      not been compromised
     */
};

#endif
