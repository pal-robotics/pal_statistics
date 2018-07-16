/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef PAL_STATISTICS_MACROS_H
#define PAL_STATISTICS_MACROS_H
#include <pal_statistics/pal_statistics.h>

#define DEFAULT_STATISTICS_TOPIC "pal_statistics"

namespace pal
{
boost::shared_ptr<pal::StatisticsRegistry> getRegistry(const std::string &topic);
} //namespace pal

#define REGISTER_VARIABLE(TOPIC, ID, VARIABLE, BOOKKEEPING)                              \
  pal::getRegistry(TOPIC)->registerVariable(ID, VARIABLE, BOOKKEEPING);

#define REGISTER_VARIABLE_BK(ID, VARIABLE, BOOKKEEPING)                                  \
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, ID, VARIABLE, BOOKKEEPING)


#define REGISTER_VARIABLE_TOPIC(TOPIC, ID, VARIABLE)                                     \
  REGISTER_VARIABLE(TOPIC, ID, VARIABLE, NULL)

#define SIMPLE_REGISTER(ID, VARIABLE)                                                    \
  REGISTER_VARIABLE_TOPIC(DEFAULT_STATISTICS_TOPIC, ID, VARIABLE)

#define PUBLISH_STATISTICS(TOPIC) pal::getRegistry(TOPIC)->publish();

#define PUBLISH_ASYNC_STATISTICS(TOPIC) pal::getRegistry(TOPIC)->publishAsync();

#define SIMPLE_PUBLISH_STATISTICS() PUBLISH_STATISTICS(DEFAULT_STATISTICS_TOPIC)

#define SIMPLE_PUBLISH_ASYNC_STATISTICS() PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)

#define START_PUBLISH_THREAD(TOPIC) pal::getRegistry(TOPIC)->startPublishThread();

#define SIMPLE_START_PUBLISH_THREAD() START_PUBLISH_THREAD(DEFAULT_STATISTICS_TOPIC)

#define PUBLISH_CUSTOM_STATISTIC_TOPIC(TOPIC, ID, VALUE) pal::getRegistry(TOPIC)->publishCustomStatistic(ID, VALUE)

#define PUBLISH_CUSTOM_STATISTIC(ID, VALUE) PUBLISH_CUSTOM_STATISTIC_TOPIC(DEFAULT_STATISTICS_TOPIC, ID, VALUE)

#define PUBLISH_CUSTOM_STATISTICS_TOPIC(TOPIC, MSG) pal::getRegistry(TOPIC)->publishCustomStatistics(MSG)

#define PUBLISH_CUSTOM_STATISTICS(MSG) PUBLISH_CUSTOM_STATISTIC_TOPIC(DEFAULT_STATISTICS_TOPIC, MSG)


#endif  // PAL_STATISTICS_MACROS_H
