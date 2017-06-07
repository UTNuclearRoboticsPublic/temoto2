#ifndef DESCRIPTION_PROCESSOR_H
#define DESCRIPTION_PROCESSOR_H

#include <tinyxml.h>            // http://www.dinomage.com/2012/01/tutorial-using-tinyxml-part-1/
#include <string>
#include <vector>

class DescriptionProcessor
{

public:


    std::string getTaskType( std::string path );

    TaskInfo getTaskInfo ( std::string path );

    int checkTaskSignature (TiXmlElement* rootElement);
};

#endif
