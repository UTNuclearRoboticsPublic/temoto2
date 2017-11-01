/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "output_manager/output_manager_interface.h"
#include "sensor_manager/sensor_manager_interface.h"


// First implementaton
class TaskStart: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaskStart()
{
    // Do something here if needed
    TASK_INFO("TaskStart constructed");
}

// startTask with arguments
bool startTask(TTP::TaskInterface task_interface)
{
// * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;
    switch(task_interface.id_)
    {
        // Interface 0
        case 0:
            startInterface_0();
        break;

        // Interface 1
        case 1:
            startInterface_1();
        break;
    }

    return true;
// * AUTO-GENERATED, DO NOT MODIFY *
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string  where_0_word_out;
    float        where_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskStart got: " << what_0_word_in << std::endl;
    std::cout << getName() << std::endl;

    where_0_word_out = what_0_word_in + "land";
    where_0_data_0_out = 12.345;

// --------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject where_0_out("where", where_0_word_out);
    where_0_out.markComplete();

    where_0_out.data_.emplace_back("number", boost::any_cast<float>(where_0_data_0_out));
    output_subjects.push_back(where_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >

}

/*
 * Interface 1 body
 */
void startInterface_1()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // Creating output variables
    std::string  what_0_word_out;
    std::string  what_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

    try
    {
        // Initialize the sensor manager interface
        smi_.initialize(this);

        TASK_INFO(" Starting the camera");

        // Start the camera with our custom launch file
        std::string camera_topic = smi_.startSensor("camera");

        TASK_INFO_STREAM("Got camera on topic '" << camera_topic << "'");

        // Pass the camera topic to the output
        what_0_word_out = what_0_word_in;
        what_0_data_0_out = camera_topic;
    }
    catch( error::ErrorStackUtil& e )
    {
        e.forward( prefix );
        this->error_handler_.append(e);
    }


// -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    TTP::Subject what_0_out("what", what_0_word_out);
    what_0_out.markComplete();
    what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
    output_subjects.push_back(what_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
}

std::string getStatus()
{
    std::string str = "healthy";
    return str;
}

std::vector<TTP::Subject> getSolution()
{
    return output_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

~TaskStart()
{
    TASK_INFO("TaskStart destructed");
}

private:

// Create sensor manager interface object for accessing sensor manager
SensorManagerInterface smi_;

std::string class_name_ = "TaskStart";

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskStart, TTP::BaseTask);
