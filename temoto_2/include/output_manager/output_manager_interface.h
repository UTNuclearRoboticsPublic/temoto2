#include "core/common.h"

#include "base_task/task_errors.h"
#include "temoto_2/showInRviz.h"
#include "temoto_2/stopAllocatedServices.h"


class OutputManagerInterface
{
public:

    OutputManagerInterface()
    {
        // Start the clients
        this->show_in_rviz_client_ = n_.serviceClient<temoto_2::getGestures>("show_in_rviz");
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
    }

    void stopAllocatedServices()
    {
        // Name of the method, used for making debugging a bit simpler
        std::string prefix = formatMessage("", this->class_name_, __func__);

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

    ~OutputManagerInterface()
    {
        // Let the context manager know, that task is finished and topics are unsubscribed
        stopAllocatedServices();
    }

private:

    std::string id_;

    const std::string class_name_ = "OutputManagerInterface";

    ros::NodeHandle n_;

    ros::ServiceClient show_in_rviz_client_;
    ros::ServiceClient stop_allocated_services_;
};
