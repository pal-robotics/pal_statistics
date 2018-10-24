/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef PAL_STATISTICS_MACROS_H
#define PAL_STATISTICS_MACROS_H
#include <pal_statistics/pal_statistics.h>
#include <pal_statistics/registration_utils.h>

constexpr char DEFAULT_STATISTICS_TOPIC[] = "pal_statistics";

namespace pal_statistics
{
boost::shared_ptr<StatisticsRegistry> getRegistry(const std::string &topic);
} //namespace pal_statistics

#define REGISTER_VARIABLE(TOPIC, ID, VARIABLE, BOOKKEEPING)                              \
  customRegister(*pal_statistics::getRegistry(TOPIC), ID, VARIABLE, BOOKKEEPING);

// Register the variable with the same name as the variable name
#define REGISTER_VARIABLE_SIMPLE(TOPIC, VARIABLE, BOOKKEEPING)                              \
  customRegister(*pal_statistics::getRegistry(TOPIC), #VARIABLE, VARIABLE, BOOKKEEPING);

#define UNREGISTER_VARIABLE(TOPIC, ID_OR_NAME, BOOKKEEPING)                              \
  pal_statistics::getRegistry(TOPIC)->unregisterVariable(ID_OR_NAME, BOOKKEEPING);

#define PUBLISH_STATISTICS(TOPIC) pal_statistics::getRegistry(TOPIC)->publish();

#define PUBLISH_ASYNC_STATISTICS(TOPIC) pal_statistics::getRegistry(TOPIC)->publishAsync();

#define START_PUBLISH_THREAD(TOPIC) pal_statistics::getRegistry(TOPIC)->startPublishThread();

#define PUBLISH_CUSTOM_STATISTIC(TOPIC, ID, VALUE) pal_statistics::getRegistry(TOPIC)->publishCustomStatistic(ID, VALUE)

#define PUBLISH_CUSTOM_STATISTICS_MSG(TOPIC, MSG) pal_statistics::getRegistry(TOPIC)->publishCustomStatistics(MSG)



#endif  // PAL_STATISTICS_MACROS_H
