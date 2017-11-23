#ifndef CONFIG_SYNCHRONIZER_H
#define CONFIG_SYNCHRONIZER_H

#include "ros/ros.h"
#include "rmp/log_macros.h"
#include <string>
#include <sstream>
#include "temoto_2/ConfigSync.h"

namespace rmp
{

namespace sync_action
{
const std::string UPDATE = "update_config";
const std::string REQUEST_CONFIG = "request_config";
}


template <class Owner>
class ConfigSynchronizer
{
public:
  typedef void (Owner::*OwnerCbType)(const temoto_2::ConfigSync&);

  ConfigSynchronizer(const std::string& name, const std::string& sync_topic, OwnerCbType sync_cb,
                     Owner* owner)
    : name_(name), sync_topic_(sync_topic), sync_cb_(sync_cb), owner_(owner)
  {
    log_class_ = "rmp/ConfigSync";
    log_subsys_ = name;
    std::string prefix = common::generateLogPrefix(log_subsys_, log_class_, "");

    // Setup publisher and subscriber
    sync_pub_ = nh_.advertise<temoto_2::ConfigSync>(sync_topic, 1000);
    sync_sub_ = nh_.subscribe(sync_topic, 1000, &ConfigSynchronizer::wrappedSyncCb, this);

    // Ask from master how many nodes have subscribed to sync_topic
    // Wait until all connections are established.
    int total_connections = 0;
    int active_connections = 0;
    while (true)
    {
      XmlRpc::XmlRpcValue args, result, payload;
      args[0] = ros::this_node::getName();
      ros::S_string node_set;
      if (ros::master::execute("getSystemState", args, result, payload, true))
      {
        for (int i = 0; i < payload.size(); ++i)
        {
          for (int j = 0; j < payload[i].size(); ++j)
          {
            std::string topic = payload[i][j][0];
            XmlRpc::XmlRpcValue nodes = payload[i][j][1];
            if (topic == sync_topic_)
            {
              total_connections = nodes.size();
            }
          }
        }
      }
      active_connections = sync_pub_.getNumSubscribers();
      RMP_DEBUG("Waiting for subscribers: %d/%d", active_connections, total_connections);
      if (active_connections == total_connections)
      {
        break;
      }
      ros::Duration(0.1).sleep();
    }

    RMP_INFO("ConfigSynchronizer created.");
  }

  ~ConfigSynchronizer()
  {
    sync_pub_.shutdown();
    sync_sub_.shutdown();
  }

  void requestRemoteConfigs()
  {
    temoto_2::ConfigSync msg;
    msg.temoto_namespace = common::getTemotoNamespace();
    msg.action = sync_action::REQUEST_CONFIG;
    msg.config = "";
    sync_pub_.publish(msg);
  }

  void sendUpdate(const YAML::Node& config)
  {
    temoto_2::ConfigSync msg;
    msg.temoto_namespace = common::getTemotoNamespace();
    msg.action = sync_action::UPDATE;
    
    msg.config = Dump(config);
    sync_pub_.publish(msg);
  }

private:

  void wrappedSyncCb(const temoto_2::ConfigSync& msg)
  {
    // Ignore messages that are ours
    if (msg.temoto_namespace == common::getTemotoNamespace())
    {
      return;
    }

    (owner_->*sync_cb_)(msg);
  }

  std::string name_;
  std::string sync_topic_;
  OwnerCbType sync_cb_;
  Owner* owner_;

  ros::NodeHandle nh_;
  ros::Publisher sync_pub_;
  ros::Subscriber sync_sub_;

  std::string log_subsys_, log_class_;

};
}

#endif
