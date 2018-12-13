#include <yaml-cpp/yaml.h>
#include "temoto_sensor_manager/sensor_info.h"

namespace temoto_sensor_manager
{
namespace sm_yaml
{
  std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config);
} // sm_yaml namespace
} // sensor_manager namespace
