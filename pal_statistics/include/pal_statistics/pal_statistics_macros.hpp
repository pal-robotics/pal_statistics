// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef PAL_STATISTICS__PAL_STATISTICS_MACROS_HPP_
#define PAL_STATISTICS__PAL_STATISTICS_MACROS_HPP_

#include <memory>
#include <string>

#include "pal_statistics/pal_statistics.hpp"
#include "pal_statistics/registration_utils.hpp"

constexpr char DEFAULT_STATISTICS_TOPIC[] = "pal_statistics";

namespace pal_statistics
{
std::shared_ptr<StatisticsRegistry> getRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & node_namespace,
  const std::string & topic);

std::shared_ptr<StatisticsRegistry> getRegistry(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic);

std::shared_ptr<StatisticsRegistry> getRegistry(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::string & topic);
}  // namespace pal_statistics


// Trick to use macros with optional argument, in practice there are three version of the macro:
// REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE, BOOKKEEPING) -> full specification of arguments
// REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE)              -> No bookkeeping

// https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros?
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

// UNREGISTER_VARIABLE(NODE, TOPIC, ID, BOOKKEEPING) -> full specification of arguments
// UNREGISTER_VARIABLE(NODE, TOPIC, ID)              -> No bookkeeping
#define UNREGISTER_VARIABLE(...) UNREGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


#endif  // PAL_STATISTICS__PAL_STATISTICS_MACROS_HPP_
