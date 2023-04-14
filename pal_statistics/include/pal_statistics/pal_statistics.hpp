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


#ifndef PAL_STATISTICS__PAL_STATISTICS_HPP_
#define PAL_STATISTICS__PAL_STATISTICS_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "pal_statistics/pal_statistics_utils.hpp"

#include "pal_statistics_msgs/msg/statistics.hpp"
#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include "pal_statistics_msgs/msg/statistics_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// fordward declaration to make it friend of StatisticsRegistry
template<typename NodeT>
class PalStatisticsTestHelperClass;

namespace pal_statistics
{

// forward declarations
template<typename T>
class LockFreeQueue;
struct EnabledId;
class RegistrationList;

/**
 * @brief The StatisticsRegistry class reads the value of registered variables and
 * publishes them on the specified topic.
 *
 * @warning Functions are not real-time safe unless stated.
 *
 * @warning Registering and enabling more than one variable with the same name is not
 * supported. Multiple variables with the same name can be registered, only if there's
 * only one of them enabled at any time.
 *
 * If you are using repeated names, it's better to use the Id of the registered variables
 * or the RegistratonRAII for unregister/disable
 */
class StatisticsRegistry : public std::enable_shared_from_this<StatisticsRegistry>
{
public:
  StatisticsRegistry(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
    const std::string & topic);

  StatisticsRegistry(const std::shared_ptr<rclcpp::Node> & node, const std::string & topic);
  StatisticsRegistry(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic);

  virtual ~StatisticsRegistry();

  /**
   * @brief registerVariable Specialization for double*, the most common case, to avoid
   * going through a boost function call to read the variable
   */
  IdType registerVariable(
    const std::string & name, const double * variable, RegistrationsRAII * bookkeeping = NULL,
    bool enabled = true);

  /**
   * @brief registerFunction Adds a function that returns double with the specified name
   * @param bookkeeping same as in registerVariable
   */
  IdType registerFunction(
    const std::string & name, const std::function<double()> & funct,
    RegistrationsRAII * bookkeeping = NULL, bool enabled = true);


  void unregisterVariable(const std::string & name, RegistrationsRAII * bookkeeping = NULL);
  void unregisterVariable(IdType id, RegistrationsRAII * bookkeeping = NULL);

  /**
   * @brief publish Reads the values of all registered variables and publishes them to the
   * topic associated to this object.
   * @warning This function may lock if another thread is adding/removing registered
   * variables, it may allocate memory, use publishAsync if you need RT safety
   */
  void publish();

  /**
   * @brief publishAsync Capture data and flag it to be published by the publisher thread.
   * Real-Time safe.
   * @warning If the mutex cannot be acquired, data will not be captured. Should
   * only happen if other threads are registering/unregistering or publishing variables.
   * @return true if mutex and data could be acquired, false otherwise
   */
  bool publishAsync();

  /**
   * @brief startPublishThread creates and starts the publisherThread. The user still has
   * to call publishAsync each time a message must be publisher.
   */
  void startPublishThread();

  /**
   * @brief createMsg creates a Statistics message from the registered variables, useful
   * for debugging
   * @return
   */
  pal_statistics_msgs::msg::Statistics createMsg();

  /**
   * @brief publishCustomStatistic publishes a one-time statistic
   */
  template<typename T>
  void publishCustomStatistic(const std::string & name, T value)
  {
    pal_statistics_msgs::msg::Statistics msg;
    pal_statistics_msgs::msg::Statistic stat;
    stat.name = name;
    stat.value = static_cast<double>(value);
    msg.statistics.push_back(stat);
    msg.header.stamp = clock_->now();
    pub_->publish(msg);
  }

  /**
   * @brief publishCustomStatistic publishes a one-time statistics msg
   */
  void publishCustomStatistics(const pal_statistics_msgs::msg::Statistics & msg)
  {
    pub_->publish(msg);
  }

  /**
   * These functions disable/enable the publication of one or more variables
   *
   * They are RT safe and thread safe.
   * They guarantee that on the next publish (or publishAsync) call, the specified variables will
   * or will not be read and published.
   *
   * @warning If a publish is being executed while this function is being run,
   * it will not take into account these modifications.
   *
   * If you need a deterministic way of preventing a variable from being published,
   * you need to unregister it, but it is not RT safe.
   */
  bool enable(const IdType & id);
  bool disable(const IdType & id);

private:
  /**
   * @brief updateMsgUnsafe Updates the internal message variable without acquiring the
   * mutex Should only be used if the mutex has already been acquired by the thread
   * calling this
   */
  void updateMsgUnsafe();

  /**
   * @brief updateMsg update names and values, optionally using smartfill to minimize copying
   * @return true if a smartfill was performed
   */
  bool updateMsg(
    pal_statistics_msgs::msg::StatisticsNames & names,
    pal_statistics_msgs::msg::StatisticsValues & values, bool smart_fill = false);

  void publisherThreadCycle();

  void startPublishThreadImpl();

  IdType registerInternal(
    const std::string & name, VariableHolder && variable,
    RegistrationsRAII * bookkeeping, bool enabled);

  bool setEnabledmpl(const IdType & id, bool enabled);

  /**
   * @brief handlePendingDisables Empties by handling the queue of disabled/enabled ids.
   */
  void handlePendingDisables(const std::unique_lock<std::mutex> & data_lock);

  /**
   * @brief doPublish publishes the subscribed topics, requires mutex
   */
  void doPublish(bool publish_names_msg = true);

  const rclcpp::Logger & getLogger() const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::mutex data_mutex_;
  std::unique_ptr<RegistrationList> registration_list_;

  /**
   * @brief disabled_ids_ this is used to keep track of enabled/disabled variables in a
   * lock free way
   *
   * enable/disable need to write, but they cannot be locked, and cannot be
   * skipped if they fail to acquire a mutex.
   * Therefore they write to a lockfree  structure.
   * This structure is processed in the next publish or publishAsync that has
   * the write lock and can modify shared structures.
   */
  std::unique_ptr<LockFreeQueue<EnabledId>> enabled_ids_;


  // To avoid deadlocks, should always be acquired after data_mutex_
  std::mutex pub_mutex_;
  rclcpp::Publisher<pal_statistics_msgs::msg::Statistics>::SharedPtr pub_;
  rclcpp::Publisher<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr pub_names_;
  rclcpp::Publisher<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr pub_values_;

  std::atomic<bool> is_data_ready_;
  std::atomic<bool> interrupt_thread_;
  std::shared_ptr<std::thread> publisher_thread_;

  struct GeneratedStatistics
  {
    GeneratedStatistics()
    : last_names_version_(-1)
    {
    }
    void update(
      const pal_statistics_msgs::msg::StatisticsNames & names,
      const pal_statistics_msgs::msg::StatisticsValues & values);

    /// This message is generated using an updated StatiticsNames and StatisticsValues
    pal_statistics_msgs::msg::Statistics msg_;
    unsigned int last_names_version_;
  };

  pal_statistics_msgs::msg::StatisticsNames names_msg_;
  pal_statistics_msgs::msg::StatisticsValues values_msg_;
  GeneratedStatistics generated_statistics_;
  // Internal stats
  unsigned int publish_async_attempts_;
  unsigned int publish_async_failures_;
  double last_async_pub_duration_;
  RegistrationsRAII internal_stats_raii_;

  template<typename NodeT>
  friend class ::PalStatisticsTestHelperClass;
};
}  // namespace pal_statistics
#endif  // PAL_STATISTICS__PAL_STATISTICS_HPP_
