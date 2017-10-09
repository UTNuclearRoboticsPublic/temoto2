#ifndef SENSOR_MANAGER_INTERFACE_H
#define SENSOR_MANAGER_INTERFACE_H

#include "core/common.h"
#include "base_task/task_errors.h"
#include "sensor_manager/sensor_manager_services.h"
#include "rmp/resource_manager.h"

class SensorManagerInterface
{
public:

    /**
     * @brief SensorManagerInterface
     */
    SensorManagerInterface() : resource_manager_("SensorManagerInterface", this)
    {
    }

    /**
     * @brief startSensor
     * @param sensor_type
     */
    std::string startSensor( std::string sensor_type )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        try
        {
            return startSensor( sensor_type, "", "" );
        }
        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            error_handler_.append(e);
        }
    }

    /**
     * @brief startSensor
     * @param sensor_type
     * @param package_name
     * @param ros_program_name
     */
    std::string startSensor( std::string sensor_type, std::string package_name, std::string ros_program_name )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        // Fill out the "StartSensorRequest" request
        temoto_2::LoadSensor srv_msg;
        srv_msg.request.sensor_type = sensor_type;
        srv_msg.request.package_name = package_name;
        srv_msg.request.executable = ros_program_name;

        // Call the server
        if( !resource_manager_.template call<temoto_2::LoadSensor>(
					sensor_manager::srv_name::MANAGER,
					sensor_manager::srv_name::SERVER,
					srv_msg) )
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        // If the request was fulfilled, then add the srv to the list of allocated sensors
        if( srv_msg.response.rmp.code == 0 )
        {
            allocated_sensors_.push_back(srv_msg);
            return srv_msg.response.topic;
        }
        else
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Unsuccessful call to sensor manager: " +
                                            srv_msg.response.rmp.message,
                                         ros::Time::now());
        }
    }

    /**
     * @brief stopSensor
     * @param sensor_type
     * @param package_name
     * @param ros_program_name
     */
    void stopSensor( std::string sensor_type, std::string package_name, std::string ros_program_name )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

		// Find all instances where request part matches of what was given and unload each resource
		temoto_2::LoadSensor::Request req;
		req.sensor_type = sensor_type;
		req.package_name = package_name;
		req.executable = ros_program_name;
        
		auto cur_sensor_it = allocated_sensors_.begin();
		while (cur_sensor_it != allocated_sensors_.end())
		{
			// The == operator used in the lambda function is defined in
			// sensor manager services header
			auto found_sensor_it = std::find_if(cur_sensor_it, allocated_sensors_.end(),
					[&](const temoto_2::LoadSensor& srv_msg) -> 
					bool {return srv_msg.request == req;}
					);
			if (found_sensor_it != allocated_sensors_.end())
			{
				// do the unloading
				resource_manager_.unloadResource(found_sensor_it->response.rmp.resource_id);
				cur_sensor_it = found_sensor_it;
			}
			else if(cur_sensor_it == allocated_sensors_.begin())
			{
				throw error::ErrorStackUtil( taskErr::RESOURCE_UNLOAD_FAIL,
						error::Subsystem::TASK,
						error::Urgency::MEDIUM,
						prefix + " Unable to unload resource that is not loaded.",
						ros::Time::now());
			}
		}
    }

    ~SensorManagerInterface()
    {
		// Name of the method, used for making debugging a bit simpler
		// std::string prefix = formatMessage("", this->class_name_, __func__);
    }

private:

    const std::string class_name_ = "SensorManagerInterface";
    error::ErrorHandler error_handler_;
    std::vector <temoto_2::LoadSensor> allocated_sensors_;
	rmp::ResourceManager<SensorManagerInterface> resource_manager_;

};

#endif
