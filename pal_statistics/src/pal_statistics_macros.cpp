/*
  @file
  
  @author victor
  
  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/pal_statistics_macros.h>
namespace pal
{
boost::shared_ptr<pal::StatisticsRegistry> getRegistry(const std::string &topic)
{
  typedef std::map<std::string, boost::shared_ptr<pal::StatisticsRegistry> > RegistryMap;
  static RegistryMap registries;

  RegistryMap::const_iterator cit = registries.find(topic);

  if (cit == registries.end())
  {
    boost::shared_ptr<pal::StatisticsRegistry> ptr =
        boost::make_shared<pal::StatisticsRegistry>(topic);
    registries[topic] = ptr;
    return ptr;
  }
  else
  {
    return cit->second;
  }
}
} //namespace pal
