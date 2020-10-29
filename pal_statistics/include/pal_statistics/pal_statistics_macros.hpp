/**
 *
 * MIT License
 *
 * Copyright (c) 2019 PAL Robotics S.L.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#ifndef PAL_STATISTICS_MACROS_HPP_
#define PAL_STATISTICS_MACROS_HPP_
#include <pal_statistics/pal_statistics.hpp>
#include <pal_statistics/registration_utils.hpp>

constexpr char DEFAULT_STATISTICS_TOPIC[] = "pal_statistics";

namespace pal_statistics
{
std::shared_ptr<StatisticsRegistry> getRegistry(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic);
} //namespace pal_statistics


//Trick to use macros with optional argument, in practice there are three version of the macro:
//REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE, BOOKKEEPING) -> full specification of arguments
//REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE)              -> No bookkeeping

//https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros?
#define REGISTER_VARIABLE_4_ARGS(NODE, TOPIC, ID, VARIABLE) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), ID, VARIABLE);
#define REGISTER_VARIABLE_5_ARGS(NODE, TOPIC, ID, VARIABLE, BOOKKEEPING) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), ID, VARIABLE, BOOKKEEPING);

#define GET_6TH_ARG(arg1, arg2, arg3, arg4, arg5, arg6, ...) arg6
#define REGISTER_MACRO_CHOOSER(...) \
  GET_6TH_ARG( \
    __VA_ARGS__, REGISTER_VARIABLE_5_ARGS, \
    REGISTER_VARIABLE_4_ARGS)

#define REGISTER_VARIABLE(...) REGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

// Register the variable with the same name as the variable name
#define REGISTER_VARIABLE_SIMPLE(NODE, TOPIC, VARIABLE, BOOKKEEPING) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), #VARIABLE, VARIABLE, BOOKKEEPING);


#define PUBLISH_STATISTICS(NODE, TOPIC) pal_statistics::getRegistry(NODE, TOPIC)->publish();

#define PUBLISH_ASYNC_STATISTICS(NODE, TOPIC) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->publishAsync();

#define START_PUBLISH_THREAD(NODE, TOPIC) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->startPublishThread();

#define PUBLISH_CUSTOM_STATISTIC(NODE, TOPIC, ID, VALUE) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->publishCustomStatistic(ID, VALUE)

#define PUBLISH_CUSTOM_STATISTICS_MSG(NODE, TOPIC, MSG) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->publishCustomStatistics(MSG)


#define UNREGISTER_VARIABLE_3_ARGS(NODE, TOPIC, ID) \
  pal_statistics::getRegistry(NODE, TOPIC)->unregisterVariable(ID);
#define UNREGISTER_VARIABLE_4_ARGS(NODE, TOPIC, ID, BOOKKEEPING) \
  pal_statistics::getRegistry(NODE, TOPIC)->unregisterVariable(ID, BOOKKEEPING);

#define GET_5TH_ARG(arg1, arg2, arg3, arg4, arg5, ...) arg5
#define UNREGISTER_MACRO_CHOOSER(...) \
  GET_5TH_ARG( \
    __VA_ARGS__, UNREGISTER_VARIABLE_4_ARGS, \
    UNREGISTER_VARIABLE_3_ARGS)

//UNREGISTER_VARIABLE(NODE, TOPIC, ID, BOOKKEEPING) -> full specification of arguments
//UNREGISTER_VARIABLE(NODE, TOPIC, ID)              -> No bookkeeping
#define UNREGISTER_VARIABLE(...) UNREGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


#endif  // PAL_STATISTICS_MACROS_HPP_
