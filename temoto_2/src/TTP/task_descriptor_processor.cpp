#include "TTP/task_descriptor_processor.h"
#include "common/tools.h"
#include "meta/analyzers/filters/porter2_stemmer.h"

#include <string>
#include <vector>
#include <boost/any.hpp>
#include <boost/filesystem/operations.hpp>

namespace TTP
{

/* * * * * * * * *
 *  PARSE STRING. A string parsing class could help
 * * * * * * * * */

std::vector<std::string> parseString (std::string in_str, char delimiter)
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

    // Extract the words
    while ( getline(iss, s, delimiter) )
    {
        strings.push_back(s);
    }

    return strings;
}


/* * * * * * * * *
 *  CONSTRUCTOR
 * * * * * * * * */

TaskDescriptorProcessor::TaskDescriptorProcessor(std::string path, BaseSubsystem& b) 
    :
      BaseSubsystem(b),
      base_path_(path)
{
    class_name_ = __func__;
    desc_file_path_ = base_path_ + "/descriptor.xml";
    try
    {
        // Open the task and get the root element
        openTaskDesc();
        getRootElement();
    }

    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
}

/* * * * * * * * *
 *  GET PACKAGE NAME
 * * * * * * * * */

std::string TaskDescriptorProcessor::getPackageName()
{
    TiXmlDocument package_xml;

    // Open the description file
    if( !package_xml.LoadFile( base_path_ + "/package.xml") )
    {
        // Throw error
        throw CREATE_ERROR(error::Code::DESC_OPEN_FAIL, std::string(package_xml.ErrorDesc()));
    }

    // Get root element
    TiXmlElement* package_el = package_xml.FirstChildElement("package");

    if( package_el == NULL )
    { 
        throw CREATE_ERROR(error::Code::DESC_NO_ROOT, "Missing element 'package'.");
    }

    // Get the name of the package
    TiXmlElement* name_el = package_el->FirstChildElement("name");
    if (name_el == NULL)
    {
        // Throw error
        throw CREATE_ERROR(error::Code::DESC_NO_ROOT, "Missing element 'name'.");
    }

    TiXmlNode* name_content = name_el->FirstChild();
    std::string name_str;
    if(name_content == NULL)
    {
        throw CREATE_ERROR(error::Code::DESC_NO_ROOT, "Content in the 'name' block is missing.");
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

void TaskDescriptorProcessor::openTaskDesc()
{
    // Open the description file
    if( !desc_file_.LoadFile(desc_file_path_) )
    {
        // Throw error
        throw CREATE_ERROR(error::Code::DESC_OPEN_FAIL, std::string( desc_file_.ErrorDesc() ) + desc_file_path_);
    }
}


/* * * * * * * * *
 *  GET ROOT ELEMENT
 * * * * * * * * */

void TaskDescriptorProcessor::getRootElement()
{
    // Get the root element
    root_element_ = desc_file_.FirstChildElement();
    if( root_element_ == NULL )
    {
        // Throw error
        throw CREATE_ERROR(error::Code::DESC_NO_ROOT, "No root element in: " + desc_file_path_);
    }
}

/* * * * * * * * *
 *  GET TASK ACTION
 * * * * * * * * */

Action TaskDescriptorProcessor::getTaskAction()
{
    // Get the type of the task
    const char * action_attribute = root_element_->Attribute("action");

    if (action_attribute == NULL)
    {      
        // Throw error
        throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing 'action' attribute in: " + desc_file_path_);
    }

    return std::move(std::string(action_attribute));
}


/* * * * * * * * *
 *  STEM ACTION
 * * * * * * * * */

Action stemAction (const Action& non_stemmed)
{
    Action stemmed = non_stemmed;
    meta::analyzers::filters::porter2::stem(stemmed);

    return std::move(stemmed);
}

/* * * * * * * * *
 *  STEM ACTIONS
 * * * * * * * * */

std::vector<Action> stemActions (const std::vector<Action>& actions)
{
    // TODO: Im more than sure that this can throw something, catch that something
    std::vector<Action> stemmed_actions;
    for (auto& action : actions)
    {
        stemmed_actions.push_back(stemAction(action));
    }

    return std::move(stemmed_actions);
}


/* * * * * * * * *
 *  GET TASK ACTION ALIASES
 * * * * * * * * */

std::vector<Action> TaskDescriptorProcessor::getTaskActionAliases()
{
    // Get the type of the task
    const char * alias_attribute = root_element_->Attribute("alias");

    if (alias_attribute == NULL)
    {
        std::vector<Action> empty_vec;
        return std::move(empty_vec);
    }

    return std::move(parseString(std::string(alias_attribute), ',')); // TODO: allow ', ' default expressions
}

/* * * * * * * * *
 *  GET TASK DESCRIPTOR
 * * * * * * * * */

TaskDescriptor TaskDescriptorProcessor::getTaskDescriptor()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    // TaskDescriptorProcessor is a friend of TaskDescriptor
    TaskDescriptor task_descriptor;

    try
    {
        task_descriptor.action_ = getTaskAction();
        task_descriptor.action_stemmed_ = stemAction(task_descriptor.action_);

        task_descriptor.aliases_ = getTaskActionAliases();
        task_descriptor.aliases_stemmed_ = stemActions(task_descriptor.aliases_);
        task_descriptor.aliases_stemmed_.push_back(task_descriptor.action_stemmed_);

        task_descriptor.task_interfaces_ = getInterfaces();
        task_descriptor.task_package_name_ = getPackageName();

        // TODO: check if this file even exists
        boost::filesystem::path path(base_path_ + "/lib/lib" + task_descriptor.task_package_name_ + ".so");
        task_descriptor.task_lib_path_ = boost::filesystem::canonical(path).string();
    }

    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    return task_descriptor;
}

/* * * * * * * * *
 *  GET INTERFACES
 * * * * * * * * */

std::vector<TaskInterface> TaskDescriptorProcessor::getInterfaces()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    std::vector<TaskInterface> task_interfaces;


    try
    {
        for (TiXmlElement* interface_element = root_element_->FirstChildElement("interface");
             interface_element != NULL;
             interface_element = interface_element->NextSiblingElement("interface"))
        {
            task_interfaces.push_back(getInterface(interface_element));
        }

        if (task_interfaces.empty())
        {
            // Throw error
            throw CREATE_ERROR(error::Code::DESC_INVALID_ARG, "Missing 'interface' elements in: " + desc_file_path_);
        }

    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
    return std::move(task_interfaces);
}

/* * * * * * * * *
 *  GET INTERFACE
 * * * * * * * * */

TaskInterface TaskDescriptorProcessor::getInterface(TiXmlElement* interface_element)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    TaskInterface task_interface;

    // Get the interface id
    const char* id_attribute = interface_element->Attribute("id");
    if (id_attribute == NULL)
    {
        throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing id attribute in: " + desc_file_path_);
    }

    task_interface.id_ = atoi(id_attribute);

    // Get the interface type
    const char* type_attribute = interface_element->Attribute("type");
    if (type_attribute == NULL)
    {
        throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing type attribute in: " + desc_file_path_);
    }

    task_interface.type_ = std::string(type_attribute);

    try
    {
        //REQUIRED
        task_interface.input_subjects_ = getIOSubjects("in", interface_element);

        //NOT REQUIRED
        task_interface.output_subjects_ = getIOSubjects("out", interface_element);
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
    return std::move(task_interface);
}

/* * * * * * * * *
 *  GET IODESCRIPTOR
 * * * * * * * * */

std::vector<Subject> TaskDescriptorProcessor::getIOSubjects(std::string direction, TiXmlElement* interface_element)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    std::vector<Subject> io_subjects;

    try
    {
        TiXmlElement* io_element = interface_element->FirstChildElement(direction);

        // Check if this element is valid.
        if (io_element == NULL)
        {
            // OUTPUT DESCRIPTOR IS NOT REQUIRED
            if (direction == "out")
            {
                return std::move(io_subjects);
            }

            // Throw error
            throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing '" + direction + "' attribute in: " + desc_file_path_);
        }

        // Extract subjects (whats, wheres, numerics)
        for (TiXmlElement* subject_element = io_element->FirstChildElement();
             subject_element != NULL;
             subject_element = subject_element->NextSiblingElement())
        {
            // Check if the subject type is valid
            std::string sub_type = std::string(subject_element->Value());
            if (std::find(valid_subjects.begin(), valid_subjects.end(), sub_type) == valid_subjects.end())
            {
                // Throw error
                throw CREATE_ERROR(error::Code::DESC_INVALID_ARG, "Invalid datatype: " + desc_file_path_);
            }

            // Parse the subject and add it to the IO-Descriptor
            Subject subject = getSubject(subject_element);
            io_subjects.push_back(subject);
        }
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    return std::move(io_subjects);

}

/* * * * * * * * *
 *  GET SUBJECT
 * * * * * * * * */

Subject TaskDescriptorProcessor::getSubject(TiXmlElement* subject_element)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    Subject subject;

    try
    {
        // Get the type
        subject.type_ = std::string(subject_element->Value());

        /*
         * Get the arg element. If it's missing, then that's not good
         */
        TiXmlElement* arg_element = subject_element->FirstChildElement("arg");

        // Check if this element is valid. REQUIRED
        if (arg_element == NULL)
        {
            // Throw error
            throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing 'arg' attribute in: " + desc_file_path_);
        }

        // Get the word attribute. REQUIRED
        const char* word_attribute = arg_element->Attribute("word");
        if ( word_attribute == NULL )
        {
            // Throw error
            throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing 'word' attribute in: " + desc_file_path_);
        }

        subject.words_ = std::move(parseString(std::string(word_attribute), ',')); // TODO: allow ', ' default expressions

        // Get the Part Of Speech Tag (pos_tag) attribute. NOT REQUIRED
        const char* pos_tag_attribute = arg_element->Attribute("pos_tag");
        if ( pos_tag_attribute != NULL )
        {
            /*
            // Throw error
            throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing 'pos_tag' attribute in: " + desc_file_path_);
            */

            subject.pos_tag_ = std::string(pos_tag_attribute);
        }

        /*
         * Get the data element. If it's missing, then that's ok. It means that there
         * is no additional data requested/provided
         */
        TiXmlElement* data_element = subject_element->FirstChildElement("data");

        // Check if this element is valid
        if (data_element == NULL)
        {
            return subject;
        }

        subject.data_ = getData(data_element);
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    return subject;
}

/* * * * * * * * *
 *  GET DATA
 * * * * * * * * */

std::vector<Data> TaskDescriptorProcessor::getData(TiXmlElement* data_element)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    std::vector<Data> datas;

    try
    {
        // Extract the datafields
        for(TiXmlElement* field_element = data_element->FirstChildElement("field");
            field_element != NULL;
            field_element = field_element->NextSiblingElement("field"))
        {
            // Get the datatype attribute. REQUIRED
            const char* datatype_attribute = field_element->Attribute("datatype");
            if ( datatype_attribute == NULL )
            {
                // Throw error
                throw CREATE_ERROR(error::Code::DESC_NO_ATTR, "Missing 'datatype' attribute in: " + desc_file_path_);
            }

            std::string datatype = std::string(datatype_attribute);

            // Check if the datatype is valid
            if (std::find(valid_datatypes.begin(), valid_datatypes.end(), datatype) == valid_datatypes.end())
            {
                // Throw error
                throw CREATE_ERROR(error::Code::DESC_INVALID_ARG, "Invalid datatype: " + desc_file_path_);
            }

            Data data;
            data.type = datatype;

            // Get the value attribute. NOT REQUIRED
            const char* value_attribute = field_element->Attribute("value");
            if ( value_attribute != NULL )
            {
                std::string value_str = std::string(value_attribute);

                if (data.type == "string")
                {
                    data.value = boost::any_cast<std::string>(value_str);
                }

                if (data.type == "topic")
                {
                    data.value = boost::any_cast<std::string>(value_str);
                }

                if (data.type == "number")
                {
                    double value_num = atof(value_str.c_str());
                    data.value = boost::any_cast<double>(value_num);
                }
            }

            datas.push_back(data);
        }
    }
    catch(error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
    catch (boost::bad_any_cast& e)
    {
        throw CREATE_ERROR(error::Code::BAD_ANY_CAST, std::string(e.what()) + ". in: " + desc_file_path_);
    }

    return datas;
}

/* * * * * * * * *
 *  CHECK DESC INTEGRITY
 * * * * * * * * */



/* * * * * * * * *
 *  DESTRUCTOR
 * * * * * * * * */

TaskDescriptorProcessor::~TaskDescriptorProcessor()
{
    desc_file_.Clear();
}

} // End of TTP namespace
