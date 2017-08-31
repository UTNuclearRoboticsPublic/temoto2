#include "sensor_manager/sensor_manager_interface.h"

/* * * * * * * * * * * * * * * * *
 *  SensorManagerInterface
 * * * * * * * * * * * * * * * * */

SensorManagerInterface::SensorManagerInterface()
{
    // Start the service clients
    start_sensor_client_ = n_.serviceClient<temoto_2::startSensorRequest>("start_sensor");
    stop_sensor_client_ = n_.serviceClient<temoto_2::stopSensorRequest>("stop_sensor");
}

/* * * * * * * * * * * * * * * * *
 *  startSensor by type
 * * * * * * * * * * * * * * * * */

std::string SensorManagerInterface::startSensor( std::string sensor_type )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    try
    {
        return startSensorPrivate( sensor_type, "", "" );
    }
    catch( error::ErrorStackUtil& e )
    {
        e.forward( prefix );
        error_handler_.append(e);
    }
}

/* * * * * * * * * * * * * * * * *
 *  startSensor by package and program
 * * * * * * * * * * * * * * * * */

std::string SensorManagerInterface::startSensor( std::string sensor_type,
                                                 std::string package_name,
                                                 std::string ros_program_name )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    try
    {
        return startSensorPrivate( sensor_type, package_name, ros_program_name );
    }
    catch( error::ErrorStackUtil& e )
    {
        e.forward( prefix );
        error_handler_.append(e);
    }
}

/* * * * * * * * * * * * * * * * *
 *  startSensorPrivate
 * * * * * * * * * * * * * * * * */

std::string SensorManagerInterface::startSensorPrivate( std::string sensor_type,
                                                        std::string package_name,
                                                        std::string ros_program_name )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Fill out the "StartSensorRequest" request
    temoto_2::startSensorRequest start_sensor_srv;
    start_sensor_srv.request.type = sensor_type;
    start_sensor_srv.request.name = package_name;
    start_sensor_srv.request.executable = ros_program_name;

    // Call the server
    if( !start_sensor_client_.call(start_sensor_srv) )
    {
        throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::TASK,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service",
                                     ros::Time::now());
    }

    // If the request was fulfilled, then add the srv to the list of allocated sensors
    if( start_sensor_client_.response.code == 0 )
    {
        allocated_sensors_.push_back(start_sensor_srv);
        return start_sensor_srv.response.topic;
    }
    else
    {
        throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::TASK,
                                     error::Urgency::MEDIUM,
                                     prefix + " Unsuccessful call to /show_in_rviz" +
                                        show_in_rviz_srv.response.message,
                                     ros::Time::now());
    }
}

/* * * * * * * * * * * * * * * * *
 *  stopSensorPrivate
 * * * * * * * * * * * * * * * * */

void SensorManagerInterface::stopSensorPrivate( std::string sensor_type,
                                                std::string package_name,
                                                std::string ros_program_name )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Fill out the "StopSensorRequest" request
    temoto_2::stopSensorRequest stop_sensor_srv;
    stop_sensor_srv.request.type = sensor_type;
    stop_sensor_srv.request.name = package_name;
    stop_sensor_srv.request.executable = ros_program_name;

    // Call the server
    if( !stop_sensor_client_.call(stop_sensor_srv) )
    {
        throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::TASK,
                                     error::Urgency::MEDIUM,
                                     prefix + " Failed to call service",
                                     ros::Time::now());
    }

    // Check If the request was fulfilled successfully
    if( stop_sensor_client_.response.code != 0 )
    {
        throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                     error::Subsystem::TASK,
                                     error::Urgency::MEDIUM,
                                     prefix + " Unsuccessful call to /show_in_rviz" +
                                        show_in_rviz_srv.response.message,
                                     ros::Time::now());
    }
}

/* * * * * * * * * * * * * * * * *
 *  stopSensor
 * * * * * * * * * * * * * * * * */
void SensorManagerInterface::stopSensor( std::string sensor_type, std::string package_name, std::string ros_program_name )
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    // Sensor can only be stopped if it was started beforehand, duh
    for( auto& allocated_sensor : allocated_sensors )
    {
        if( allocated_sensor.request.type == sensor_type &&
            allocated_sensor.response.name == package_name &&
            allocated_sensor.response.executable == ros_program_name)
        {
            try
            {
                stopSensorPrivate( sensor_type, package_name, ros_program_name );
            }
            catch( error::ErrorStackUtil& e )
            {
                e.forward( prefix );
                error_handler_.append(e);
            }
        }
    }
}

/* * * * * * * * * * * * * * * * *
 *  ~SensorManagerInterface
 * * * * * * * * * * * * * * * * */

SensorManagerInterface::~SensorManagerInterface()
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = formatMessage("", this->class_name_, __func__);

    for( auto& allocated_sensor : allocated_sensors )
    {
        try
        {
            stopSensorPrivate( allocated_sensor.request.type,
                               allocated_sensor.response.name,
                               allocated_sensor.reponse.executable);
        }
        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            error_handler_.append(e);
        }
    }
}
