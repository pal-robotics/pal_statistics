/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include "pal_statistics/pal_statistics_macros.hpp"

#include <map>
#include <memory>
#include <string>

namespace pal_statistics
{
std::shared_ptr<StatisticsRegistry> getRegistry(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic)
{
  typedef std::map<std::string, std::shared_ptr<StatisticsRegistry>> RegistryMap;
  static RegistryMap registries;

  RegistryMap::const_iterator cit = registries.find(node->get_effective_namespace() + topic);

  if (cit == registries.end()) {
    std::shared_ptr<StatisticsRegistry> ptr =
      std::make_shared<StatisticsRegistry>(node, topic);
    registries[node->get_effective_namespace() + topic] = ptr;
    return ptr;
  } else {
    return cit->second;
  }
}
}  // namespace pal_statistics
