#include "core/common.h"

#include "common/temoto_id.h"
#include "base_task/task_errors.h"
#include "output_manager/output_manager_errors.h"
#include "temoto_2/ShowInRviz.h"
#include "temoto_2/stopAllocatedServices.h"
#include <sstream>
#include <fstream>

/**
 * @brief The OutputManagerInterface class
 */
class OutputManagerInterface
{
public:

    /**
     * @brief OutputManagerInterface
     */
    OutputManagerInterface()
    {
        // Start the clients
        this->show_in_rviz_client_ = nh_.serviceClient<temoto_2::ShowInRviz>("show_in_rviz");
        this->stop_allocated_services_ = nh_.serviceClient<temoto_2::stopAllocatedServices>("stop_allocated_services_om");
    }

    /**
     * @brief showInRviz
     * @param display_type
     * @param topic
     * @param display_config
     */
    void showInRviz( std::string display_type, std::string topic = "", std::string display_config = "" )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        temoto_2::ShowInRviz show_in_rviz_srv;
        show_in_rviz_srv.request.type = display_type;
        show_in_rviz_srv.request.topic = topic;
        show_in_rviz_srv.request.config = display_config;

        // Call the server
        if( !show_in_rviz_client_.call(show_in_rviz_srv) )
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to call service",
                                         ros::Time::now());
        }

        if( show_in_rviz_srv.response.code == 0 )
        {
            id_ = show_in_rviz_srv.response.id;
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

    /**
     * @brief stopAllocatedServices
     */
    void stopAllocatedServices()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        if( id_ != TemotoID::UNASSIGNED_ID )
        {
            temoto_2::stopAllocatedServices stopSrv;
            stopSrv.request.id = id_;

            // Call the server
            if (!stop_allocated_services_.call(stopSrv))
            {
                throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                             error::Subsystem::TASK,
                                             error::Urgency::HIGH,
                                             prefix + " Failed to call service",
                                             ros::Time::now());
            }
        }
        else
        {
            ROS_INFO("%s This interface has not allocated any resources.", prefix.c_str());
        }
    }

    /**
     * @brief displayConfigFromFile
     * @param config_path
     * @return
     */
    std::string displayConfigFromFile( std::string config_path )
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        // Create filestream object and configure exceptions
        std::ifstream config_file;
        config_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        try
        {
            // Open the file stream
            config_file.open( config_path );

            // Stream the file into a stringstream
            std::stringstream sstr;
            while(config_file >> sstr.rdbuf());

            return sstr.str();
        }
        catch (std::ifstream::failure e)
        {
            // Rethrow the exception
            throw error::ErrorStackUtil( outputManagerErr::FILE_OPEN_FAIL,
                                         error::Subsystem::TASK,
                                         error::Urgency::MEDIUM,
                                         prefix + " Failed to open the display config file",
                                         ros::Time::now() );
        }
    }

    ~OutputManagerInterface()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        // Let the context manager know, that task is finished and topics are unsubscribed
        try
        {
            stopAllocatedServices();
        }
        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            error_handler_.append(e);
        }
    }

private:

    TemotoID::ID id_ = TemotoID::UNASSIGNED_ID;

    const std::string class_name_ = "OutputManagerInterface";

    ros::NodeHandle nh_;

    error::ErrorHandler error_handler_;

    ros::ServiceClient show_in_rviz_client_;
    ros::ServiceClient stop_allocated_services_;
};
