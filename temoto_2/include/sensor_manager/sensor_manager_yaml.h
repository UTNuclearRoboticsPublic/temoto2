#include <yaml-cpp/yaml.h>
#include "sensor_manager/sensor_info.h"

namespace sensor_manager
{
namespace sm_yaml
{
  std::vector<SensorInfoPtr> parseSensors(const YAML::Node& config);
} // sm_yaml namespace
} // sensor_manager namespace
