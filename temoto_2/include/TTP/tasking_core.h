#ifndef TASKING_CORE_H
#define TASKING_CORE_H

#include "ros/ros.h"
#include "ros/package.h"

#include "TTP/language_processors/meta/meta_lp.h"
#include "TTP/task_manager.h"
#include "std_msgs/String.h"

namespace TTP
{

class TaskingCore
{
public:

    TaskingCore(std::string node_name);

private:

    ros::NodeHandle nh_;

    ros::Subscriber human_chatter_subscriber_;

    TTP::TaskManager task_manager_;

    TTP::MetaLP* language_processor_;

    void humanChatterCb (std_msgs::String chat);

    std::string class_name_ = "TaskingCore";

};

} // End of TTP namespace

#endif
