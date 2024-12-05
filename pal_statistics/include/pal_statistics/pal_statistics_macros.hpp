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
/**
 * @brief Creates the registry for the given node interfaces and topic and stores it in the
 * global registry map using the key, if it doesn't exist. If it already exists, it returns the
 * existing one.
 *
 * @param parameters_interface - The parameters interface of the node
 * @param topics_interface - The topics interface of the node
 * @param logging_interface - The logging interface of the node
 * @param clock_interface - The clock interface of the node
 * @param topic - The topic to publish the statistics
 * @param registry_key - The key to store the registry in the global registry map
 *
*/
std::shared_ptr<StatisticsRegistry> getOrCreateRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & topic, const std::string & registry_key);

/**
 * @brief Creates the registry for the given node and topic and stores it in the global registry
 * map using the key, if it doesn't exist. If it already exists, it returns the existing one.
 * @param node - The node to create the registry
 * @param topic - The topic to publish the statistics
 * @param key - The key to store the registry in the global registry map
 * @return std::shared_ptr<StatisticsRegistry> - The registry stored in the global registry map
 * using the key
 */
template<typename NodeT>
std::shared_ptr<StatisticsRegistry> getOrCreateRegistry(
  const NodeT & node,
  const std::string & topic, const std::string & custom_key)
{
  return getOrCreateRegistry(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic, custom_key);
}

/**
 * @brief Returns the unique registry key for the given node and topic.
 * @param node - The node to create the registry
 * @param topic - The topic to publish the statistics
*/
template<typename NodeT>
std::string getUniqueRegistryKey(const NodeT & node, const std::string & topic)
{
  return node->get_node_topics_interface()->resolve_topic_name(topic);
}

/**
 * @brief Returns the registry stored in the global registry map using the key.
 * If it doesn't exist, it returns a nullptr.
 * @param registry_key - The key to check the registry in the global registry map
 * @return std::shared_ptr<StatisticsRegistry> - The registry stored in the global registry map
 * using the key
 * @return nullptr - If the registry doesn't exist
 */
std::shared_ptr<StatisticsRegistry> getRegistry(const std::string & registry_key);

/**
 * @brief Deletes the registry stored in the global registry map using the key.
 * @param registry_key - The key to delete the registry in the global registry map
 *
 * @note This function is not real-time safe
 * @note If the registry doesn't exist, it does nothing
 */
void deleteRegistry(const std::string & registry_key);

/**
 * @brief Deletes all the registries stored in the global registry map.
 * @note This function is not real-time safe
 */
void clearAllRegistries();

[[deprecated("Use getOrCreateRegistry(parameters_interface, topics_interface, logging_interface, "
             "clock_interface, topic, registry_key) instead")]]
std::shared_ptr<StatisticsRegistry> getRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & node_namespace,
  const std::string & topic);

template<typename NodeT>
std::shared_ptr<StatisticsRegistry> getRegistry(
  const NodeT & node, const std::string & topic)
{
  return getOrCreateRegistry(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic,
    getUniqueRegistryKey(node, topic));
}
}  // namespace pal_statistics


#define INITIALIZE_REGISTRY_2_ARGS(NODE, TOPIC) \
  pal_statistics::getOrCreateRegistry( \
    NODE, TOPIC, \
    NODE->get_node_topics_interface()->resolve_topic_name(TOPIC))

#define INITIALIZE_REGISTRY_3_ARGS(NODE, TOPIC, CUSTOM_KEY) \
  pal_statistics::getOrCreateRegistry(NODE, TOPIC, CUSTOM_KEY)

#define GET_4TH_ARG(arg1, arg2, arg3, arg4, ...) arg4
#define INITIALIZE_MACRO_CHOOSER(...) \
  GET_4TH_ARG( \
    __VA_ARGS__, INITIALIZE_REGISTRY_3_ARGS, \
    INITIALIZE_REGISTRY_2_ARGS)

#define INITIALIZE_REGISTRY(...) INITIALIZE_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define DELETE_REGISTRY_1_ARGS(REGISTRY_KEY) \
  pal_statistics::deleteRegistry(REGISTRY_KEY)

#define DELETE_REGISTRY_2_ARGS(NODE, TOPIC) \
  pal_statistics::deleteRegistry(pal_statistics::getUniqueRegistryKey(NODE, TOPIC))

#define GET_3TH_ARG(arg1, arg2, arg3, ...) arg3
#define DELETE_REGISTRY_MACRO_CHOOSER(...) \
  GET_3TH_ARG( \
    __VA_ARGS__, DELETE_REGISTRY_2_ARGS, \
    DELETE_REGISTRY_1_ARGS)

#define DELETE_REGISTRY(...) DELETE_REGISTRY_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define CLEAR_ALL_REGISTRIES() \
  pal_statistics::clearAllRegistries()

#define REGISTER_ENTITY_3_ARGS(REGISTRY_KEY, ID, ENTITY) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::customRegister(*pal_statistics::getRegistry(REGISTRY_KEY), ID, ENTITY); \
  } else { \
    RCLCPP_WARN_STREAM( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to register entity " << ID << " in " << REGISTRY_KEY << \
        ", as the registry is not found. Try initializing it!"); \
  }

#define REGISTER_ENTITY_4_ARGS(REGISTRY_KEY, ID, ENTITY, BOOKKEEPING) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::customRegister( \
      *pal_statistics::getRegistry(REGISTRY_KEY), ID, ENTITY, BOOKKEEPING); \
  } else { \
    RCLCPP_WARN_STREAM( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to register entity " << ID << " in " << REGISTRY_KEY << \
        ", as the registry is not found. Try initializing it!"); \
  }

#define REGISTER_ENTITY_5_ARGS(REGISTRY_KEY, ID, ENTITY, BOOKKEEPING, ENABLE) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::customRegister( \
      *pal_statistics::getRegistry(REGISTRY_KEY), ID, ENTITY, BOOKKEEPING, ENABLE); \
  } else { \
    RCLCPP_WARN_STREAM( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to register entity " << ID << " in " << REGISTRY_KEY << \
        ", as the registry is not found. Try initializing it!"); \
  }

#define GET_6TH_ARG(arg1, arg2, arg3, arg4, arg5, arg6, ...) arg6
#define REGISTER_ENTITY_MACRO_CHOOSER(...) \
  GET_6TH_ARG( \
    __VA_ARGS__, REGISTER_ENTITY_5_ARGS, REGISTER_ENTITY_4_ARGS, \
    REGISTER_ENTITY_3_ARGS)

#define REGISTER_ENTITY(...) REGISTER_ENTITY_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define UNREGISTER_ENTITY_2_ARGS(REGISTRY_KEY, ID) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::getRegistry(REGISTRY_KEY)->unregisterVariable(ID); \
  } else { \
    RCLCPP_WARN_STREAM( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to unregister entity " << ID << " in " << REGISTRY_KEY << \
        ", as the registry is not found."); \
  }

#define UNREGISTER_ENTITY_3_ARGS(REGISTRY_KEY, ID, BOOKKEEPING) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::getRegistry(REGISTRY_KEY)->unregisterVariable(ID, BOOKKEEPING); \
  } else { \
    RCLCPP_WARN_STREAM( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to unregister entity " << ID << " in " << REGISTRY_KEY << \
        ", as the registry is not found."); \
  }

#define UNREGISTER_ENTITY_MACRO_CHOOSER(...) \
  GET_4TH_ARG( \
    __VA_ARGS__, UNREGISTER_ENTITY_3_ARGS, \
    UNREGISTER_ENTITY_2_ARGS)

#define UNREGISTER_ENTITY(...) UNREGISTER_ENTITY_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

// Trick to use macros with optional argument, in practice there are three version of the macro:
// REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE, BOOKKEEPING) -> full specification of arguments
// REGISTER_VARIABLE(NODE, TOPIC, ID, VARIABLE)              -> No bookkeeping

// https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros?
#define REGISTER_VARIABLE_4_ARGS(NODE, TOPIC, ID, VARIABLE) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), ID, VARIABLE);
#define REGISTER_VARIABLE_5_ARGS(NODE, TOPIC, ID, VARIABLE, BOOKKEEPING) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), ID, VARIABLE, BOOKKEEPING);

#define REGISTER_MACRO_CHOOSER(...) \
  GET_6TH_ARG( \
    __VA_ARGS__, REGISTER_VARIABLE_5_ARGS, \
    REGISTER_VARIABLE_4_ARGS)

#define REGISTER_VARIABLE(...) REGISTER_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

// Register the variable with the same name as the variable name
#define REGISTER_VARIABLE_SIMPLE(NODE, TOPIC, VARIABLE, BOOKKEEPING) \
  customRegister(*pal_statistics::getRegistry(NODE, TOPIC), #VARIABLE, VARIABLE, BOOKKEEPING);


#define PUBLISH_STATISTICS_1_ARGS(REGISTRY_KEY) \
  pal_statistics::getRegistry(REGISTRY_KEY)->publish();
#define PUBLISH_STATISTICS_2_ARGS(NODE, TOPIC) \
  pal_statistics::getRegistry(NODE, TOPIC)->publish();

#define PUBLISH_STATISTICS_MACRO_CHOOSER(...) \
  GET_3TH_ARG( \
    __VA_ARGS__, PUBLISH_STATISTICS_2_ARGS, \
    PUBLISH_STATISTICS_1_ARGS)

#define PUBLISH_STATISTICS(...) PUBLISH_STATISTICS_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


#define PUBLISH_ASYNC_STATISTICS_1_ARGS(REGISTRY_KEY) \
  if (pal_statistics::getRegistry(REGISTRY_KEY) != nullptr) { \
    pal_statistics::getRegistry(REGISTRY_KEY)->publishAsync(); \
  } else { \
    RCLCPP_WARN_STREAM_ONCE( \
      rclcpp::get_logger("pal_statistics"), \
      "Unable to publish async statistics in " << REGISTRY_KEY << \
        ", as the registry is not found."); \
  }
#define PUBLISH_ASYNC_STATISTICS_2_ARGS(NODE, TOPIC) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->publishAsync();
#define PUBLISH_ASYNC_STATISTICS_MACRO_CHOOSER(...) \
  GET_3TH_ARG( \
    __VA_ARGS__, PUBLISH_ASYNC_STATISTICS_2_ARGS, \
    PUBLISH_ASYNC_STATISTICS_1_ARGS)

#define PUBLISH_ASYNC_STATISTICS(...) PUBLISH_ASYNC_STATISTICS_MACRO_CHOOSER(__VA_ARGS__)( \
    __VA_ARGS__)

#define START_PUBLISH_THREAD_1_ARGS(REGISTRY_KEY) pal_statistics::getRegistry( \
    REGISTRY_KEY)->startPublishThread();
#define START_PUBLISH_THREAD_2_ARGS(NODE, TOPIC) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->startPublishThread();

#define START_PUBLISH_THREAD_MACRO_CHOOSER(...) \
  GET_3TH_ARG( \
    __VA_ARGS__, START_PUBLISH_THREAD_2_ARGS, \
    START_PUBLISH_THREAD_1_ARGS)

#define START_PUBLISH_THREAD(...) START_PUBLISH_THREAD_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define STOP_PUBLISHER_THREAD_1_ARGS(REGISTRY_KEY) pal_statistics::getRegistry( \
    REGISTRY_KEY)->stopPublisherThread();
#define STOP_PUBLISHER_THREAD_2_ARGS(NODE, TOPIC) pal_statistics::getRegistry( \
    NODE, \
    TOPIC)->stopPublisherThread();

#define STOP_PUBLISHER_THREAD_MACRO_CHOOSER(...) \
  GET_3TH_ARG( \
    __VA_ARGS__, STOP_PUBLISHER_THREAD_2_ARGS, \
    STOP_PUBLISHER_THREAD_1_ARGS)

#define STOP_PUBLISHER_THREAD(...) STOP_PUBLISHER_THREAD_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

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
