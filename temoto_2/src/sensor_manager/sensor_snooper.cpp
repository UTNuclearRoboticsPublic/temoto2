#include "sensor_manager/sensor_snooper.h"
#include "sensor_manager/sensor_manager_services.h"

#include "ros/package.h"
#include <yaml-cpp/yaml.h>


namespace sensor_manager
{

// TODO: the constructor of the action_engine_ can throw in the initializer list
//       and I have no clue what kind of behaviour should be expected - prolly bad

SensorSnooper::SensorSnooper( BaseSubsystem*b
                            , SensorInfoDatabase* sid)
: BaseSubsystem(*b)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &SensorSnooper::syncCb, this)
, action_engine_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/../actions/object_tracker_actions")
, sid_(sid)
{
  // Get the name of this class
  class_name_ = __func__;

  // Run the sensor snooping actions
  startSnooping();

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

//  std::string action = "track";
//  TTP::Subjects subjects;

//  // Subject that will contain the name of the tracked object.
//  // Necessary when the tracker has to be stopped
//  TTP::Subject sub_0("what", object_name_no_space);

//  // Subject that will contain the data necessary for the specific tracker
//  TTP::Subject sub_1("what", "artag data");

//  // Topic from where the raw AR tag tracker data comes from
//  sub_1.addData("topic", tracker_data_topic);

//  // Topic where the AImp must publish the data about the tracked object
//  sub_1.addData("topic", tracked_object_topic);

//  // This object will be updated inside the tracking AImp (action implementation)
//  sub_1.addData("pointer", boost::any_cast<ObjectPtr>(requested_object));

//  subjects.push_back(sub_0);
//  subjects.push_back(sub_1);

//  // Create a SF
//  std::vector<TTP::TaskDescriptor> task_descriptors;
//  task_descriptors.emplace_back(action, subjects);
//  task_descriptors[0].setActionStemmed(action);

//  // Create a sematic frame tree
//  TTP::TaskTree sft = TTP::SFTBuilder::build(task_descriptors);

//  // Get the root node of the tree
//  TTP::TaskTreeNode& root_node = sft.getRootNode();
//  sft.printTaskDescriptors(root_node);

//  // Execute the SFT
//  tracker_core_.executeSFT(std::move(sft));
}

void SensorSnooper::advertiseSensor(SensorInfoPtr sensor_ptr) const
{
  //TEMOTO_DEBUG("------ Advertising Sensor \n %s", sensor_ptr->toString().c_str());
  YAML::Node config;
  config["Sensors"].push_back(*sensor_ptr);
  PayloadType payload;
  payload.data = Dump(config);
  config_syncer_.advertise(payload);
}

void SensorSnooper::advertiseLocalSensors() const
{
  // publish all local sensors
  YAML::Node config;
  for(const auto& s : sid_->getLocalSensors())
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
    advertiseLocalSensors();
    return;
  }

  if (msg.action == rmp::sync_action::ADVERTISE_CONFIG)
  {
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
      if (sid_->updateRemoteSensor(*sensor))
      {
        TEMOTO_DEBUG("Updating remote sensor '%s' at '%s'.", sensor->getName().c_str(),
                     sensor->getTemotoNamespace().c_str());
      }
      else
      {
        sid_->addRemoteSensor(*sensor);
      }
    }
  }
}

} // sensor_manager namespace
