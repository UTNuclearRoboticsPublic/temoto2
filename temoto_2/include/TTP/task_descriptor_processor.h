#ifndef TASK_DESCRIPTOR_PROCESSOR_H
#define TASK_DESCRIPTOR_PROCESSOR_H

#include "TTP/task_descriptor.h"
#include <tinyxml.h>            // http://www.dinomage.com/2012/01/tutorial-using-tinyxml-part-1/

namespace TTP
{

class TaskDescriptorProcessor
{

public:

    TaskDescriptorProcessor( std::string path );

    std::string getTaskName();

    TaskDescriptor getTaskDescriptor();

    int checkTaskSignature();

    ~TaskDescriptorProcessor();

private:

    // Params
    std::string class_name_ = "TaskDescriptorProcessor";

    std::string base_path_;

    std::string desc_file_path_;

    TiXmlDocument desc_file_;

    TiXmlElement* root_element_;

    // Methods

    void openTaskDesc();

    void getRootElement();

    std::string getTaskPath();

    std::string getPackageName();


    std::vector<TaskInterface> getInterfaces();

    TaskInterface getInterface(TiXmlElement* interface_element);

    std::vector<Subject> getIOSubjects(std::string direction, TiXmlElement* interface_element);

    Subject getSubject(TiXmlElement* subject_element);

    std::vector<Data> getData(TiXmlElement* data_element);



    /* TODO:
     *
     * TASK VALIDATOR METHOD - checks that there are no colliding
     *      input or output args, also calculates the checksum
     *      for making sure that the integrity of the task has
     *      not been compromised
     */
};

} // END of TTP namespace
#endif
