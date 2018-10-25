#include "algorithm_manager/algorithm_snooper.h"
#include "algorithm_manager/algorithm_manager_services.h"

#include "ros/package.h"
#include <yaml-cpp/yaml.h>


namespace algorithm_manager
{

// TODO: the constructor of the action_engine_ can throw in the initializer list
//       and I have no clue what kind of behaviour should be expected - prolly bad

AlgorithmSnooper::AlgorithmSnooper( BaseSubsystem*b
                            , AlgorithmInfoRegistry* aid)
: BaseSubsystem(*b, __func__)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &AlgorithmSnooper::syncCb, this)
, action_engine_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/../temoto_actions/resource_snooper_actions")
, aid_(aid)
{
  // Algorithm Info update monitoring timer
  update_monitoring_timer_ = nh_.createTimer(ros::Duration(1), &AlgorithmSnooper::updateMonitoringTimerCb, this);

  // Get remote algorithm_infos
  config_syncer_.requestRemoteConfigs();
}

void AlgorithmSnooper::startSnooping()
{
  /*
   * Action related stuff up ahead: A semantic frame is manually created. Based on that SF
   * a SF tree is created, given that an action implementation, that corresponds to the
   * manually created SF, exists. The specific tracker task is started and it continues
   * running in the background until its ordered to be stopped.
   */

  std::string action = "find";
  TTP::Subjects subjects;

  // Subject that will contain the name of the tracked object.
  // Necessary when the tracker has to be stopped
  TTP::Subject sub_0("what", "algorithm packages");

  // Topic from where the raw AR tag tracker data comes from
  std::string catkin_ws = ros::package::getPath(ROS_PACKAGE_NAME) + "/../..";
  sub_0.addData("string", catkin_ws);

  // This object will be updated inside the tracking AImp (action implementation)
  sub_0.addData("pointer", boost::any_cast<AlgorithmInfoRegistry*>(aid_));

  subjects.push_back(sub_0);

  // Create a SF
  std::vector<TTP::TaskDescriptor> task_descriptors;
  task_descriptors.emplace_back(action, subjects);
  task_descriptors[0].setActionStemmed(action);

  // Create a sematic frame tree
  TTP::TaskTree sft = TTP::SFTBuilder::build(task_descriptors);

  // Get the root node of the tree
  TTP::TaskTreeNode& root_node = sft.getRootNode();
  sft.printTaskDescriptors(root_node);

  // Execute the SFT
  action_engine_.executeSFTThreaded(std::move(sft));
}

void AlgorithmSnooper::advertiseAlgorithm(AlgorithmInfo& ai) const
{
  //TEMOTO_DEBUG("------ Advertising Algorithm \n %s", algorithm_ptr->toString().c_str());
  YAML::Node config;
  config["Algorithms"].push_back(ai);
  PayloadType payload;
  payload.data = Dump(config);
  config_syncer_.advertise(payload);
}

void AlgorithmSnooper::advertiseLocalAlgorithms() const
{
  // publish all local algorithms
  YAML::Node config;
  for(const auto& s : aid_->getLocalAlgorithms())
  {
    config["Algorithms"].push_back(s);
  }

  // send to other managers if there is anything to send
  if(config.size())
  {
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
  }
}

std::vector<AlgorithmInfoPtr> AlgorithmSnooper::parseAlgorithms(const YAML::Node& config)
{
  std::vector<AlgorithmInfoPtr> algorithms;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Algorithms' key from config.");
    return algorithms;
  }

  YAML::Node algorithms_node = config["Algorithms"];
  if (!algorithms_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of algorithms.");
    return algorithms;
  }

  TEMOTO_DEBUG("Parsing %lu algorithms.", algorithms_node.size());

  // go over each algorithm node in the sequence
  for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the algorithm. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      AlgorithmInfo algorithm = node_it->as<AlgorithmInfo>();
      if (std::count_if(algorithms.begin(), algorithms.end(),
                        [&](const AlgorithmInfoPtr& s) { return *s == algorithm; }) == 0)
      {
        // OK, this is unique pointer, add it to the algorithms vector.
        algorithms.emplace_back(std::make_shared<AlgorithmInfo>(algorithm));
        //TEMOTO_DEBUG_STREAM("####### PARSED SENSOR: #######\n" << algorithms.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of algorithm '%s'.", algorithm.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<AlgorithmInfo> e)
    {
      TEMOTO_WARN("Failed to parse AlgorithmInfo from config.");
      continue;
    }
  }
  return algorithms;
}

void AlgorithmSnooper::syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload)
{

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseLocalAlgorithms();
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(payload.data);
    std::vector<AlgorithmInfoPtr> algorithms = parseAlgorithms(config);

    // TODO: Hold remote stuff in a map or something keyed by namespace
    // TODO: Temoto namespace can (doesn't have to) be contained in config
    for (auto& s : algorithms)
    {
      s->setTemotoNamespace(msg.temoto_namespace);
    }

    //for (auto& s : remote_algorithms_)
    //{
    //  TEMOTO_DEBUG("---------REMOTE SENSOR: \n %s", s->toString().c_str());
    //}

    for (auto& algorithm : algorithms)
    {
      // Check if algorithm has to be added or updated
      if (aid_->updateRemoteAlgorithm(*algorithm))
      {
        TEMOTO_DEBUG("Updating remote algorithm '%s' at '%s'.", algorithm->getName().c_str(),
                     algorithm->getTemotoNamespace().c_str());
      }
      else
      {
        aid_->addRemoteAlgorithm(*algorithm);
      }
    }
  }
}

void AlgorithmSnooper::updateMonitoringTimerCb(const ros::TimerEvent& e)
{

  // Iterate through local algorithms and check if their reliability has been updated
  for (const auto algorithm : aid_->getLocalAlgorithms())
  {
    if (!algorithm.getAdvertised())
    {
      AlgorithmInfo si = algorithm;
      aid_->updateLocalAlgorithm(si, true);
      advertiseAlgorithm(si);
    }
  }
}

} // algorithm_manager namespace
