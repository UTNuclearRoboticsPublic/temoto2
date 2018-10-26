#include "sensor_manager/sensor_snooper.h"
#include "sensor_manager/sensor_manager_services.h"
#include "sensor_manager/sensor_manager_yaml.h"

#include "ros/package.h"
#include <yaml-cpp/yaml.h>


namespace sensor_manager
{

// TODO: the constructor of the action_engine_ can throw in the initializer list
//       and I have no clue what kind of behaviour should be expected - prolly bad

SensorSnooper::SensorSnooper( BaseSubsystem*b
                            , SensorInfoRegistry* sir)
: BaseSubsystem(*b, __func__)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &SensorSnooper::syncCb, this)
, action_engine_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/../temoto_actions/resource_snooper_actions")
, sir_(sir)
{
  // Sensor Info update monitoring timer
  update_monitoring_timer_ = nh_.createTimer(ros::Duration(1), &SensorSnooper::updateMonitoringTimerCb, this);

  // Get remote sensor_infos
  config_syncer_.requestRemoteConfigs();
}

void SensorSnooper::startSnooping()
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
  TTP::Subject sub_0("what", "sensor packages");

  // Topic from where the raw AR tag tracker data comes from
  std::string catkin_ws = ros::package::getPath(ROS_PACKAGE_NAME) + "/../..";
  sub_0.addData("string", catkin_ws);

  // This object will be updated insire the tracking action
  sub_0.addData("pointer", boost::any_cast<SensorInfoRegistry*>(sir_));

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

void SensorSnooper::advertiseSensor(SensorInfo& si) const
{
  //TEMOTO_DEBUG("------ Advertising Sensor \n %s", sensor_ptr->toString().c_str());
  YAML::Node config;
  config["Sensors"].push_back(si);
  PayloadType payload;
  payload.data = Dump(config);
  config_syncer_.advertise(payload);
}

void SensorSnooper::advertiseLocalSensors() const
{
  // publish all local sensors
  YAML::Node config;
  for(const auto& s : sir_->getLocalSensors())
  {
    config["Sensors"].push_back(s);
  }

  // send to other managers if there is anything to send
  if(config.size())
  {
    PayloadType payload;
    payload.data = Dump(config);
    config_syncer_.advertise(payload);
  }
}

std::vector<SensorInfoPtr> SensorSnooper::parseSensors(const YAML::Node& config)
{
  std::vector<SensorInfoPtr> sensors;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Sensors' key from config.");
    return sensors;
  }

  YAML::Node sensors_node = config["Sensors"];
  if (!sensors_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of sensors.");
    return sensors;
  }

  TEMOTO_DEBUG("Parsing %lu sensors.", sensors_node.size());

  // go over each sensor node in the sequence
  for (YAML::const_iterator node_it = sensors_node.begin(); node_it != sensors_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the sensor. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      SensorInfo sensor = node_it->as<SensorInfo>();
      if (std::count_if(sensors.begin(), sensors.end(),
                        [&](const SensorInfoPtr& s) { return *s == sensor; }) == 0)
      {
        // OK, this is unique pointer, add it to the sensors vector.
        sensors.emplace_back(std::make_shared<SensorInfo>(sensor));
        //TEMOTO_DEBUG_STREAM("####### PARSED SENSOR: #######\n" << sensors.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of sensor '%s'.", sensor.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<SensorInfo> e)
    {
      TEMOTO_WARN("Failed to parse SensorInfo from config.");
      continue;
    }
  }
  return sensors;
}

void SensorSnooper::syncCb(const temoto_2::ConfigSync& msg, const PayloadType& payload)
{

  if (msg.action == rmp::sync_action::REQUEST_CONFIG)
  {
    std::cout << "Received a request to advertise local sensors" << std::endl;
    advertiseLocalSensors();
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
    std::cout << "Received a request to add or update remote sensors" << std::endl;

    // Convert the config string to YAML tree and parse
    YAML::Node config = YAML::Load(payload.data);
    std::vector<SensorInfoPtr> sensors = parseSensors(config);

    // TODO: Hold remote stuff in a map or something keyed by namespace
    // TODO: Temoto namespace can (doesn't have to) be contained in config
    for (auto& s : sensors)
    {
      s->setTemotoNamespace(msg.temoto_namespace);
    }

    //for (auto& s : remote_sensors_)
    //{
    //  TEMOTO_DEBUG("---------REMOTE SENSOR: \n %s", s->toString().c_str());
    //}

    for (auto& sensor : sensors)
    {
      // Check if sensor has to be added or updated
      if (sir_->updateRemoteSensor(*sensor))
      {
        TEMOTO_DEBUG("Updating remote sensor '%s' at '%s'.", sensor->getName().c_str(),
                     sensor->getTemotoNamespace().c_str());
      }
      else
      {
        sir_->addRemoteSensor(*sensor);
      }
    }
  }
}

void SensorSnooper::updateMonitoringTimerCb(const ros::TimerEvent& e)
{

  // Iterate through local sensors and check if their reliability has been updated
  for (const auto sensor : sir_->getLocalSensors())
  {
    if (!sensor.getAdvertised())
    {
      SensorInfo si = sensor;
      sir_->updateLocalSensor(si, true);
      advertiseSensor(si);
    }
  }
}

SensorSnooper::~SensorSnooper()
{
  TEMOTO_INFO("in the destructor of Sensor Snooper");
}

} // sensor_manager namespace
