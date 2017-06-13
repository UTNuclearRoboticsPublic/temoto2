#include "core/task_handler/description_processor.h"


/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

DescriptionProcessor::DescriptionProcessor( std::string path)
    :
      descFilePath_(path)
{
    try
    {
        // Open the task and get the root element
        openTaskDesc();
        getRootElement();
    }

    catch( error::ErrorStack& e )
    {
        // Rethrow the error
        e.emplace_back(coreErr::FORWARDING, "[DescriptionProcessor/Constructor]");
        throw e;
    }
}


/* * * * * * * * *
 *  OPEN DESC XML
 * * * * * * * * */

void DescriptionProcessor::openTaskDesc()
{
    // Open the description file
    if( !this->descFile_.LoadFile(this->descFilePath_) )
    {
        // Throw error
        error::ErrorStack errorStack;
        errorStack.emplace_back( coreErr::DESC_OPEN_FAIL, this->descFile_.ErrorDesc(), ros::Time::now() );
        throw errorStack;
    }
}


/* * * * * * * * *
 *  GET ROOT ELEMENT
 * * * * * * * * */

void DescriptionProcessor::getRootElement()
{
    // Get the root element
    this->rootElement_ = this->descFile_.FirstChildElement();
    if( this->rootElement_ == NULL )
    {
        // Throw error
        error::ErrorStack errorStack;
        errorStack.emplace_back( coreErr::DESC_NO_ROOT, "[DescriptionProcessor/getTaskType] No root element in: " + this->descFilePath_, ros::Time::now() );
        throw errorStack;
    }
}

/* * * * * * * * *
 *  GET TASK TYPE
 * * * * * * * * */

std::string DescriptionProcessor::getTaskName()
{
    // Get the type of the task
    const char * attr = this->rootElement_->Attribute("type");

    if (attr == NULL)
    {      
        // Throw error
        error::ErrorStack errorStack;
        errorStack.emplace_back( coreErr::DESC_NO_ATTR, "[DescriptionProcessor/getTaskType] Missing a 'name' attribute in: " + this->descFilePath_, ros::Time::now() );
        throw errorStack;
    }

    std::string taskName(attr);
    return taskName;
}


/* * * * * * * * *
 *  GET ARGS
 * * * * * * * * */

ParamList DescriptionProcessor::getArgs( std::string direction )
{
    ParamList paramList;

    for( TiXmlElement* elem = this->rootElement_->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement() )
    {
        std::string elemName = elem->Value();
        std::vector <std::string> args;

        // Get input arguments
        if( elemName.compare(direction) == 0 )
        {
            // Go through the arguments
            for( TiXmlElement* arg = elem->FirstChildElement(); arg != NULL; arg = arg->NextSiblingElement() )
            {
                // Get the value of "argtype" attribute
                const char * argtypeAttr = arg->Attribute("argtype");

                // If "argtype" is misspelled, throw error
                if ( argtypeAttr == NULL )
                {
                    // Throw error
                    error::ErrorStack errorStack;
                    errorStack.emplace_back( coreErr::DESC_NO_ATTR, "[DescriptionProcessor/getArgs] Missing an 'argtype' attribute in: " + this->descFilePath_, ros::Time::now() );
                    throw errorStack;
                }

                // If were good, proceed by converting the argAttr to string and push it in the "args" vector
                std::string argtypeStr(argtypeAttr);
                args.push_back(argtypeStr);

                // If argtype is "string", then get the expected values
                if( argtypeStr.compare("string") == 0 )
                {
                    const char * valueAttr = arg->Attribute("value");

                    // If "argtype" is misspelled or missing, throw error
                    if ( valueAttr == NULL )
                    {
                        // Throw error
                        error::ErrorStack errorStack;
                        errorStack.emplace_back( coreErr::DESC_NO_ATTR, "[DescriptionProcessor/getArgs] Missing a 'value' attribute in: " + this->descFilePath_, ros::Time::now() );
                        throw errorStack;
                    }

                    // If were good, proceed by converting the valueAttr to string and push it in the "args" vector
                    std::string valueStr(valueAttr);
                    args.push_back(valueStr);
                }

            }

            // Check if the in/out elements are empty or not
            if( args.size() > 0 )
            {
                // Push the arguments in paramList
                paramList.push_back( args );
            }

            else
            {
                // Throw error
                error::ErrorStack errorStack;
                errorStack.emplace_back( coreErr::DESC_INVALID_ARG, "[DescriptionProcessor/getArgs] Invalid argument in: " + this->descFilePath_, ros::Time::now() );
                throw errorStack;
            }
        }
    }

    // Check if param list is empty or not
    if( paramList.size() <= 0 )
    {
        // Throw error
        error::ErrorStack errorStack;
        errorStack.emplace_back( coreErr::DESC_INVALID_ARG, "[DescriptionProcessor/getArgs] Invalid argument in: " + this->descFilePath_, ros::Time::now() );
        throw errorStack;
    }

    return paramList;
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

    catch( error::ErrorStack& e )
    {
        // Rethrow the error
        e.emplace_back(coreErr::FORWARDING, "[DescriptionProcessor/getInputArgs]");
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

    catch( error::ErrorStack& e )
    {
        // Rethrow the error
        e.emplace_back(coreErr::FORWARDING, "[DescriptionProcessor/getOutputArgs]");
        throw e;
    }
}


/* * * * * * * * *
 *  GET TASK INFO
 * * * * * * * * */

TaskInfo DescriptionProcessor::getTaskInfo()
{
    // DescriptionProcessor is a friend of TaskInfo
    TaskInfo taskInfo;

    try
    {
        taskInfo.name_ = getTaskName();
        taskInfo.path_ = this->descFilePath_;
        taskInfo.args_ = getInputArgs();
        taskInfo.return_ = getOutputArgs();
    }

    catch( error::ErrorStack& e )
    {
        // Rethrow the error
        e.emplace_back(coreErr::FORWARDING, "[DescriptionProcessor/getTaskInfo]");
        throw e;
    }

    return taskInfo;
}


/* * * * * * * * *
 *  CHECK DESC INTEGRITY
 * * * * * * * * */



/* * * * * * * * *
 *  DESTRUCTOR
 * * * * * * * * */

DescriptionProcessor::~DescriptionProcessor()
{
    this->descFile_.Clear();
}
