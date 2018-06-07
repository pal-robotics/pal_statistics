/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef PAL_STATISTICS_MACROS_H
#define PAL_STATISTICS_MACROS_H
#include <pal_statistics/pal_statistics.h>

#define DEFAULT_STATISTICS_TOPIC "pal_statistics"

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

#define REGISTER_VARIABLE(TOPIC, ID, VARIABLE, BOOKKEEPING)                              \
  getRegistry(TOPIC)->registerVariable(ID, VARIABLE, BOOKKEEPING);

#define REGISTER_VARIABLE_BK(ID, VARIABLE, BOOKKEEPING)                                  \
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, ID, VARIABLE, BOOKKEEPING)


#define REGISTER_VARIABLE_TOPIC(TOPIC, ID, VARIABLE)                                     \
  REGISTER_VARIABLE(TOPIC, ID, VARIABLE, NULL)

#define SIMPLE_REGISTER(ID, VARIABLE)                                                    \
  REGISTER_VARIABLE_TOPIC(DEFAULT_STATISTICS_TOPIC, ID, VARIABLE)

#define PUBLISH_STATISTICS(TOPIC) getRegistry(TOPIC)->publish();

#define PUBLISH_ASYNC_STATISTICS(TOPIC) getRegistry(TOPIC)->publishAsync();

#define START_PUBLISH_THREAD(TOPIC) getRegistry(TOPIC)->startPublishThread();

#endif  // PAL_STATISTICS_MACROS_H
