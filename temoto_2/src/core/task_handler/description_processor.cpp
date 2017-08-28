#include "core/task_handler/description_processor.h"


/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

DescriptionProcessor::DescriptionProcessor( std::string path)
    :
      base_path_(path)
{
    this->desc_file_path_ = this->base_path_ + "/description.xml";
    try
    {
        // Open the task and get the root element
        openTaskDesc();
        getRootElement();
    }

    catch( error::ErrorStackUtil& e )
    {
        // Rethrow the error
        e.forward( "[DescriptionProcessor/Constructor]") ;
        throw e;
    }
}

/* * * * * * * * *
 *  GET PACKAGE NAME
 * * * * * * * * */

std::string DescriptionProcessor::getPackageName()
{
    TiXmlDocument package_xml;

    // Open the description file
    if( !package_xml.LoadFile( this->base_path_ + "/package.xml") )
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_OPEN_FAIL,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getPackageName] " + std::string( package_xml.ErrorDesc() ),
                                                ros::Time::now() );
        throw error_stack_util;
    }

    // Get root element
    TiXmlElement* package_el = package_xml.FirstChildElement("package");

    if( package_el == NULL )
    { 
        error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ROOT,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getPackageName] No package element in",
                                                ros::Time::now() );
        throw error_stack_util;
    }

    // Get the name of the package
    TiXmlElement* name_el = package_el->FirstChildElement("name");
    if (name_el == NULL)
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ROOT,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getPackageName] 'Name' element either missing or broken",
                                                ros::Time::now() );
        throw error_stack_util;
    }

    TiXmlNode* name_content = name_el->FirstChild();
    std::string name_str;
   	if(name_content == NULL)
	{
        error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ROOT,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getPackageName] Content between the 'Name' tags is missing missing or broken",
                                                ros::Time::now() );
        throw error_stack_util;
	}
	else
	{
		name_str = name_content->ValueStr();
	}

	//Now that we have the string copied, we are safe to clear the package_xml
    package_xml.Clear();

    return name_str;
}


/* * * * * * * * *
 *  OPEN DESC XML
 * * * * * * * * */

void DescriptionProcessor::openTaskDesc()
{
    // Open the description file
    if( !this->desc_file_.LoadFile(this->desc_file_path_) )
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_OPEN_FAIL,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/openTaskDesc] " + std::string( this->desc_file_.ErrorDesc() ) + this->desc_file_path_,
                                                ros::Time::now() );
        throw error_stack_util;
    }
}


/* * * * * * * * *
 *  GET ROOT ELEMENT
 * * * * * * * * */

void DescriptionProcessor::getRootElement()
{
    // Get the root element
    this->root_element_ = this->desc_file_.FirstChildElement();
    if( this->root_element_ == NULL )
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ROOT,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getTaskType] No root element in: " + this->desc_file_path_,
                                                ros::Time::now() );;
        throw error_stack_util;
    }
}

/* * * * * * * * *
 *  GET TASK TYPE
 * * * * * * * * */

std::string DescriptionProcessor::getTaskName()
{
    // Get the type of the task
    const char * attr = this->root_element_->Attribute("name");

    if (attr == NULL)
    {      
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ATTR,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getTaskType] Missing a 'name' attribute in: " + this->desc_file_path_,
                                                ros::Time::now() );
        throw error_stack_util;
    }

    std::string task_name(attr);
    return task_name;
}


/* * * * * * * * *
 *  GET ARGS
 * * * * * * * * */

ParamList DescriptionProcessor::getArgs( std::string direction )
{
    ParamList param_list;

    for( TiXmlElement* elem = this->root_element_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement() )
    {
        std::string elem_name = elem->Value();
        std::vector<ArgWithValues> args;

        // Get input arguments
        if( elem_name.compare(direction) == 0 )
        {
            // Go through the arguments
            for( TiXmlElement* arg = elem->FirstChildElement(); arg != NULL; arg = arg->NextSiblingElement() )
            {
                // Get the value of "argtype" attribute
                const char * argtype_attr = arg->Attribute("argtype");

                // If "argtype" is misspelled, throw error
                if ( argtype_attr == NULL )
                {
                    // Throw error
                    error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ATTR,
                                                            error::Subsystem::CORE,
                                                            error::Urgency::LOW,
                                                            "[DescriptionProcessor/getArgs] Missing an 'argtype' attribute in: " + this->desc_file_path_,
                                                            ros::Time::now() );
                    throw error_stack_util;
                }

                // If we are good, proceed by converting the argAttr to string and push it in the "args" vector
                std::string argtype_str(argtype_attr);
                ArgWithValues arg_with_values;
                arg_with_values.first = argtype_str;

                // Check if the argument is expecting some specific values
                const char * value_attr = arg->Attribute("value");

                // If "argtype" is misspelled or missing, throw error
                if ( value_attr == NULL )
                {
                    // Throw error
                    error::ErrorStackUtil error_stack_util( coreErr::DESC_NO_ATTR,
                                                            error::Subsystem::CORE,
                                                            error::Urgency::LOW,
                                                            "[DescriptionProcessor/getArgs] Missing a 'value' attribute in: " + this->desc_file_path_,
                                                            ros::Time::now() );
                    throw error_stack_util;


                }

                // If we are good, proceed by converting the valueAttr to string and push it in the "args" vector
                std::string value_str(value_attr);
                arg_with_values.second = parseString(value_str, ',');
                args.push_back(arg_with_values);
            }

            // Check if the in/out elements are empty or not
            if( args.size() > 0 )
            {
                // Push the arguments in paramList
                param_list.push_back( args );
            }

            else
            {
                // Throw error
                error::ErrorStackUtil error_stack_util( coreErr::DESC_INVALID_ARG,
                                                        error::Subsystem::CORE,
                                                        error::Urgency::LOW,
                                                        "[DescriptionProcessor/getArgs] Invalid argument in: " + this->desc_file_path_,
                                                        ros::Time::now() );
                throw error_stack_util;
            }
        }
    }

    // Check if param list is empty or not
    if( param_list.size() <= 0 )
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( coreErr::DESC_INVALID_ARG,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                "[DescriptionProcessor/getArgs] This file is either missing 'in' or 'out' args block: " + this->desc_file_path_,
                                                ros::Time::now() );
        throw error_stack_util;
    }

    return param_list;
}


/* * * * * * * * *
 *  GET INPUT ARGS
 * * * * * * * * */

ParamList DescriptionProcessor::getInputArgs()
{
    try
    {
        return getArgs("in");
    }

    catch( error::ErrorStackUtil& e )
    {
        // Rethrow the error
        e.forward( "[DescriptionProcessor/getInputArgs]" );
        throw e;
    }
}


/* * * * * * * * *
 *  GET OUTPUT ARGS
 * * * * * * * * */

ParamList DescriptionProcessor::getOutputArgs()
{
    try
    {
        return getArgs("out");
    }

    catch( error::ErrorStackUtil& e )
    {
        // Rethrow the error
        e.forward( "[DescriptionProcessor/getOutputArgs]" );
        throw e;
    }
}


/* * * * * * * * *
 *  GET TASK INFO
 * * * * * * * * */

TaskInfo DescriptionProcessor::getTaskInfo()
{
    // DescriptionProcessor is a friend of TaskInfo
    TaskInfo task_info;

    try
    {
        task_info.name_ = getTaskName();
        task_info.path_ = this->desc_file_path_;
        task_info.args_ = getInputArgs();
        task_info.return_ = getOutputArgs();
        task_info.package_name_ = getPackageName();
        task_info.lib_path_ = this->base_path_ + "/lib/lib" + task_info.package_name_ + ".so"; // TODO: check if this file even exists
    }

    catch( error::ErrorStackUtil& e )
    {
        // Rethrow the error
        e.forward( "[DescriptionProcessor/getTaskInfo]" );
        throw e;
    }

    return task_info;
}


/* * * * * * * * *
 *  PARSE STRING. A string parsing class could help
 * * * * * * * * */

std::vector<std::string> DescriptionProcessor::parseString (std::string in_str, char delimiter)
{
    // Create a vector for storing sentences
    std::vector <std::string> strings;

    // Create a stringstream and temporary string
    std::stringstream iss(in_str);
    std::string s;

    // TEMPORARY HACK THING for getting strings separated by whitespace
    if (delimiter == ' ')
    {
        for( ; iss >> s; )
        {
            strings.push_back(s);
        }
        return strings;
    }

    // Extract the sentences
    while ( getline(iss, s, delimiter) )
    {
        strings.push_back(s);
    }

    return strings;
}



/* * * * * * * * *
 *  CHECK DESC INTEGRITY
 * * * * * * * * */



/* * * * * * * * *
 *  DESTRUCTOR
 * * * * * * * * */

DescriptionProcessor::~DescriptionProcessor()
{
    this->desc_file_.Clear();
}
