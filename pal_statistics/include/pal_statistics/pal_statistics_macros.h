/**
 * Copyright (C) 2019 PAL Robotics S.L.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 **/
#ifndef PAL_STATISTICS_MACROS_H
#define PAL_STATISTICS_MACROS_H
#include <pal_statistics/pal_statistics.h>
#include <pal_statistics/registration_utils.h>

constexpr char DEFAULT_STATISTICS_TOPIC[] = "pal_statistics";

namespace pal_statistics
{
boost::shared_ptr<StatisticsRegistry> getRegistry(const std::string &topic);
} //namespace pal_statistics


//Trick to use macros with optional argument, in practice there are three version of the macro:
//REGISTER_VARIABLE(TOPIC, ID, VARIABLE, BOOKKEEPING) -> full specification of arguments
//REGISTER_VARIABLE(TOPIC, ID, VARIABLE)              -> No bookkeeping

//https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros?
#define REGISTER_VARIABLE_3_ARGS(TOPIC, ID, VARIABLE)                              \
  customRegister(*pal_statistics::getRegistry(TOPIC), ID, VARIABLE);
#define REGISTER_VARIABLE_4_ARGS(TOPIC, ID, VARIABLE, BOOKKEEPING)                              \
  customRegister(*pal_statistics::getRegistry(TOPIC), ID, VARIABLE, BOOKKEEPING);

#define GET_5TH_ARG(arg1, arg2, arg3, arg4, arg5, ...) arg5
#define REGISTER_MACRO_CHOOSER(...) \
    GET_5TH_ARG(__VA_ARGS__, REGISTER_VARIABLE_4_ARGS, \
                REGISTER_VARIABLE_3_ARGS)

#define REGISTER_VARIABLE(...) REGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

// Register the variable with the same name as the variable name
#define REGISTER_VARIABLE_SIMPLE(TOPIC, VARIABLE, BOOKKEEPING)                              \
  customRegister(*pal_statistics::getRegistry(TOPIC), #VARIABLE, VARIABLE, BOOKKEEPING);


#define PUBLISH_STATISTICS(TOPIC) pal_statistics::getRegistry(TOPIC)->publish();

#define PUBLISH_ASYNC_STATISTICS(TOPIC) pal_statistics::getRegistry(TOPIC)->publishAsync();

#define START_PUBLISH_THREAD(TOPIC) pal_statistics::getRegistry(TOPIC)->startPublishThread();

#define PUBLISH_CUSTOM_STATISTIC(TOPIC, ID, VALUE) pal_statistics::getRegistry(TOPIC)->publishCustomStatistic(ID, VALUE)

#define PUBLISH_CUSTOM_STATISTICS_MSG(TOPIC, MSG) pal_statistics::getRegistry(TOPIC)->publishCustomStatistics(MSG)


#define UNREGISTER_VARIABLE_2_ARGS(TOPIC, ID)                                            \
  pal_statistics::getRegistry(TOPIC)->unregisterVariable(ID);
#define UNREGISTER_VARIABLE_3_ARGS(TOPIC, ID, BOOKKEEPING)                               \
  pal_statistics::getRegistry(TOPIC)->unregisterVariable(ID, BOOKKEEPING);

#define GET_4TH_ARG(arg1, arg2, arg3, arg4, ...) arg4
#define UNREGISTER_MACRO_CHOOSER(...) \
    GET_4TH_ARG(__VA_ARGS__, UNREGISTER_VARIABLE_3_ARGS, \
                UNREGISTER_VARIABLE_2_ARGS)

//UNREGISTER_VARIABLE(TOPIC, ID, BOOKKEEPING) -> full specification of arguments
//UNREGISTER_VARIABLE(TOPIC, ID)              -> No bookkeeping
#define UNREGISTER_VARIABLE(...) UNREGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


#endif  // PAL_STATISTICS_MACROS_H
