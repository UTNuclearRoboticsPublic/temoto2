#include "sensor_manager/sensor_manager_yaml.h"

namespace sensor_manager
{
namespace sm_yaml
{

std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config)
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

} // sm_yaml namespace

} // sensor_manager namespace
