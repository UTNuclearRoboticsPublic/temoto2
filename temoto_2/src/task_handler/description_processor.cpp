#include "temoto_2/task_handler/description_processor.h"


std::string DescriptionProcessor::getTaskType( std::string path )
{
    // Open the description file
    TiXmlDocument doc;
    if(!doc.LoadFile(path))
    {
        std::cerr << "[DescriptionProcessor/processXML] " << doc.ErrorDesc() << std::endl;
        return std::string();
    }

    // Get the root element
    TiXmlElement* root = doc.FirstChildElement();
    if(root == NULL)
    {
        std::cerr << "[DescriptionProcessor/processXML] Failed to load file: No root element." << std::endl;
        doc.Clear();
        return std::string();
    }

    // Get the type of the task
    const char * attr = root->Attribute("type");

    if (attr == NULL)
    {
        std::cerr << "[DescriptionProcessor/processXML] The following file is missing a 'type' attribute: " << path << std::endl;
        doc.Clear();
        return std::string();
    }

    std::string taskType(attr);
    return taskType;
}


/* * * * * * * * *
 *  PROCESS DESC
 * * * * * * * * */

int DescriptionProcessor::processDesc (std::string taskType, std::string path)
{
    // Open the description file
    TiXmlDocument doc;
    if(!doc.LoadFile(path))
    {
        std::cerr << "[DescriptionProcessor/processXML] " << doc.ErrorDesc() << std::endl;
        return 1;
    }

    // Get the root element
    TiXmlElement* root = doc.FirstChildElement();
    if(root == NULL)
    {
        std::cerr << "[DescriptionProcessor/processXML] Failed to load file: No root element." << std::endl;
        doc.Clear();
        return 1;
    }

    // Get the type of the task
    const char * attr = root->Attribute("type");

    if (attr == NULL)
    {
        std::cerr << "[DescriptionProcessor/processXML] The following file is missing a 'type' attribute: " << path << std::endl;
        doc.Clear();
        return 1;
    }

    // Check if the requested type matches with the current type
    std::string taskTypeLocal(attr);
    if ( (taskType.compare(taskTypeLocal) == 0) || (taskType.compare("anytask") == 0) )
    {
        // std::cout << "[DescriptionProcessor/processXML] Task '" << taskType << "' found in: " << path << std::endl;
        doc.Clear();
        return 0;
    }

    // free memory
    doc.Clear();
    return 1;
}


/* * * * * * * * *
 *  CHECK DESC INTEGRITY
 * * * * * * * * */

int checkDescIntegrity (TiXmlElement* rootElement)
{
    // Check the number of required arguments


    // start reading the child elements
    for(TiXmlElement* elem = rootElement->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
    {
        // get the name of the element
        std::string elemName = elem->Value();
        std::cout << elemName << std::endl;

        // print the value of the "attribute" contained in the element
        const char* attr;
        attr = elem->Attribute("attribute");

        if (attr != NULL)
        {
           std::string attribute(attr);
           std::cout << "  " << attribute << std::endl;
        }

        // print the text if there is any
        TiXmlText* text = elem->ToText();
        if(text != NULL)
        {
            std::string t = text->Value();
            std::cout << "  " << t << std::endl;
        }
    }
}

