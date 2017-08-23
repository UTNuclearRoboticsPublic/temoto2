#include "core/common.h"

#include "common/temoto_id.h"
#include "base_task/task_errors.h"
#include "temoto_2/showInRviz.h"
#include "temoto_2/stopAllocatedServices.h"


class OutputManagerInterface
{
public:

    OutputManagerInterface()
    {
        // Start the clients
        this->show_in_rviz_client_ = n_.serviceClient<temoto_2::showInRviz>("show_in_rviz");
        this->stop_allocated_services_ = n_.serviceClient<temoto_2::stopAllocatedServices>("stop_allocated_services_om");
    }

    void showInRviz (std::string type, std::string topic)
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        temoto_2::showInRviz show_in_rviz_srv;
        show_in_rviz_srv.request.type = type;
        show_in_rviz_srv.request.topic = topic;

        // Call the server
        if (!show_in_rviz_client_.call(show_in_rviz_srv))
        {
            throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                         error::Subsystem::CORE,
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
                                         error::Subsystem::CORE,
                                         error::Urgency::MEDIUM,
                                         prefix + " Unsuccessful call to /show_in_rviz" +
                                            show_in_rviz_srv.response.message,
                                         ros::Time::now());
        }

    }

    void stopAllocatedServices()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

        if( id_ != unassigned_ID )
        {
            temoto_2::stopAllocatedServices stopSrv;
            stopSrv.request.id = id_;

            // Call the server
            if (!stop_allocated_services_.call(stopSrv))
            {
                throw error::ErrorStackUtil( taskErr::SERVICE_REQ_FAIL,
                                             error::Subsystem::CORE,
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

    TemotoID id_ = unassigned_ID;

    const std::string class_name_ = "OutputManagerInterface";

    ros::NodeHandle n_;

    error::ErrorHandler error_handler_;

    ros::ServiceClient show_in_rviz_client_;
    ros::ServiceClient stop_allocated_services_;
};
