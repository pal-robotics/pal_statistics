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


#include "pal_statistics/pal_statistics.hpp"

#include <memory>
#include <string>
#include <utility>

#include "lock_free_queue.hpp"
#include "registration_list.hpp"
#include "pal_statistics/registration_utils.hpp"

#include "rclcpp/create_publisher.hpp"

namespace pal_statistics
{

struct EnabledId
{
  // Can't use a pair because it's not trivially copiable
  IdType id;
  bool enabled;
};

StatisticsRegistry::StatisticsRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & topic)
: logger_(logging_interface->get_logger().get_child("pal_statistics")),
  clock_(clock_interface->get_clock()),
  registration_list_(new RegistrationList(logger_, clock_)),
  enabled_ids_(new LockFreeQueue<EnabledId>())
{
  pub_ = rclcpp::create_publisher<pal_statistics_msgs::msg::Statistics>(
    parameters_interface, topics_interface, topic + "/full", rclcpp::QoS(
      rclcpp::KeepAll()));

  rclcpp::QoS names_qos{rclcpp::KeepAll()};
  names_qos.reliable();
  names_qos.transient_local();  // latch

  pub_names_ = rclcpp::create_publisher<pal_statistics_msgs::msg::StatisticsNames>(
    parameters_interface, topics_interface, topic + "/names", names_qos);
  pub_values_ = rclcpp::create_publisher<pal_statistics_msgs::msg::StatisticsValues>(
    parameters_interface, topics_interface, topic + "/values", rclcpp::QoS(rclcpp::KeepAll()));

  publish_async_attempts_ = 0;
  publish_async_failures_ = 0;
  last_async_pub_duration_ = 0.0;
  interrupt_thread_ = false;
  is_data_ready_ = false;

  customRegister(
    *this, "topic_stats." + topic + ".publish_async_attempts",
    &publish_async_attempts_, &internal_stats_raii_);
  customRegister(
    *this, "topic_stats." + topic + ".publish_async_failures",
    &publish_async_failures_, &internal_stats_raii_);
  customRegister(
    *this, "topic_stats." + topic + ".publish_buffer_full_errors",
    &registration_list_->overwritten_data_count_, &internal_stats_raii_);
  customRegister(
    *this, "topic_stats." + topic + ".last_async_pub_duration",
    &last_async_pub_duration_, &internal_stats_raii_);
}

StatisticsRegistry::StatisticsRegistry(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic)
: StatisticsRegistry(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic)
{
}

StatisticsRegistry::StatisticsRegistry(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & topic)
: StatisticsRegistry(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic)
{
}

StatisticsRegistry::~StatisticsRegistry()
{
  is_data_ready_ = true;  // To let the thread exit nicely

  if (publisher_thread_) {
    interrupt_thread_ = true;
    publisher_thread_->join();
  }
  RCLCPP_INFO_STREAM(
    getLogger(), "Async messages lost " << registration_list_->overwritten_data_count_);
  RCLCPP_INFO_STREAM(getLogger(), "publish_async_failures_ " << publish_async_failures_);
}

IdType StatisticsRegistry::registerVariable(
  const std::string & name, const double * variable,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  return registerInternal(name, VariableHolder(variable), bookkeeping, enabled);
}

IdType StatisticsRegistry::registerFunction(
  const std::string & name,
  const std::function<double()> & funct,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  return registerInternal(name, VariableHolder(funct), bookkeeping, enabled);
}

void StatisticsRegistry::unregisterVariable(IdType id, RegistrationsRAII * bookkeeping)
{
  if (bookkeeping) {
    bookkeeping->remove(id);
  }

  std::unique_lock<std::mutex> data_lock(data_mutex_);
  registration_list_->unregisterVariable(id);
}

void StatisticsRegistry::unregisterVariable(
  const std::string & name,
  RegistrationsRAII * bookkeeping)
{
  if (bookkeeping) {
    bookkeeping->remove(name);
  }

  std::unique_lock<std::mutex> data_lock(data_mutex_);
  registration_list_->unregisterVariable(name);
}


void StatisticsRegistry::publish()
{
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  handlePendingDisables(data_lock);
  registration_list_->doUpdate();

  std::unique_lock<std::mutex> pub_lock(pub_mutex_);
  bool minor_changes = updateMsg(names_msg_, values_msg_, true);
  data_lock.unlock();  // msg_ is covered by pub_mutex_
  doPublish(!minor_changes);
}

bool StatisticsRegistry::publishAsync()
{
  auto begin = std::chrono::steady_clock::now();
  publish_async_attempts_++;
  if (data_mutex_.try_lock()) {
    if (!publisher_thread_.get()) {
      RCLCPP_WARN(
        getLogger(),
        "Called publishAsync but publisher thread has not been started,"
        " THIS IS NOT RT safe. You should start it yourself.");
      startPublishThreadImpl();
    }

    {
      std::unique_lock<std::mutex> data_lock(data_mutex_, std::adopt_lock);
      handlePendingDisables(data_lock);

      registration_list_->doUpdate();
    }
    is_data_ready_ = true;

    last_async_pub_duration_ = rclcpp::Duration(std::chrono::steady_clock::now() - begin).seconds();
    return true;
  }
  publish_async_failures_++;
  // Commented for RT safety
  // ROS_DEBUG("Missed publishRT opportunity because lock could not be acquired.");
  return false;
}

void StatisticsRegistry::startPublishThread()
{
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  startPublishThreadImpl();
}
void StatisticsRegistry::startPublishThreadImpl()
{
  publisher_thread_.reset(new std::thread(&StatisticsRegistry::publisherThreadCycle, this));
}

IdType StatisticsRegistry::registerInternal(
  const std::string & name, VariableHolder && variable,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  IdType id;
  {
    std::unique_lock<std::mutex> data_lock(data_mutex_);
    id = registration_list_->registerVariable(name, std::move(variable), enabled);
    enabled_ids_->set_capacity(registration_list_->size());
  }

  if (bookkeeping) {
    bookkeeping->add(Registration(name, id, weak_from_this()));
  }
  return id;
}

bool StatisticsRegistry::setEnabledmpl(const IdType & id, bool enabled)
{
  EnabledId aux;
  aux.enabled = enabled;
  aux.id = id;

  return enabled_ids_->bounded_push(aux);
}

void StatisticsRegistry::handlePendingDisables(const std::unique_lock<std::mutex> & data_lock)
{
  if (!data_lock.owns_lock() || data_lock.mutex() != &data_mutex_) {
    throw std::runtime_error("Called handlePendingDisables without proper lock");
  }

  EnabledId elem;
  while (enabled_ids_->pop(elem)) {
    registration_list_->setEnabled(elem.id, elem.enabled);
  }
}

void StatisticsRegistry::doPublish(bool publish_names_msg)
{
  if (pub_->get_subscription_count() > 0) {
    generated_statistics_.update(names_msg_, values_msg_);
    pub_->publish(generated_statistics_.msg_);
  }

  // We don't check subscribers here, because this topic is latched and we
  // always want the latest version published
  if (publish_names_msg) {  // only publish strings if changed
    pub_names_->publish(names_msg_);
  }
  if (pub_values_->get_subscription_count() > 0) {  // only publish strings if changed
    pub_values_->publish(values_msg_);
  }
}

const rclcpp::Logger & StatisticsRegistry::getLogger() const
{
  return logger_;
}

pal_statistics_msgs::msg::Statistics StatisticsRegistry::createMsg()
{
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  handlePendingDisables(data_lock);
  registration_list_->doUpdate();
  GeneratedStatistics gen_sts;
  pal_statistics_msgs::msg::StatisticsNames names;
  pal_statistics_msgs::msg::StatisticsValues values;

  updateMsg(names, values, false);
  gen_sts.update(names, values);
  return gen_sts.msg_;
}

bool StatisticsRegistry::enable(const IdType & id)
{
  return setEnabledmpl(id, true);
}

bool StatisticsRegistry::disable(const IdType & id)
{
  return setEnabledmpl(id, false);
}

bool StatisticsRegistry::updateMsg(
  pal_statistics_msgs::msg::StatisticsNames & names,
  pal_statistics_msgs::msg::StatisticsValues & values,
  bool smart_fill)
{
  if (smart_fill) {
    return registration_list_->smartFillMsg(names, values);
  } else {
    registration_list_->fillMsg(names, values);
    return false;
  }
}

void StatisticsRegistry::publisherThreadCycle()
{
  rclcpp::WallRate rate(2000);
  while (rclcpp::ok() && !interrupt_thread_) {
    while (!is_data_ready_ && !interrupt_thread_) {
      rate.sleep();
    }

    std::unique_lock<std::mutex> data_lock(data_mutex_);

    while (registration_list_->hasPendingData()) {
      bool minor_changes = updateMsg(names_msg_, values_msg_, true);

      std::unique_lock<std::mutex> pub_lock(pub_mutex_);
      data_lock.unlock();
      doPublish(!minor_changes);
      pub_lock.unlock();
      data_lock.lock();
    }
    is_data_ready_ = false;
  }
}

void StatisticsRegistry::GeneratedStatistics::update(
  const pal_statistics_msgs::msg::StatisticsNames & names,
  const pal_statistics_msgs::msg::StatisticsValues & values)
{
  msg_.header = values.header;
  if (last_names_version_ == names.names_version && !msg_.statistics.empty()) {
    // only need to update the values
    for (size_t i = 0; i < values.values.size(); ++i) {
      msg_.statistics[i].value = values.values[i];
    }
  } else {
    msg_.statistics.clear();
    for (size_t i = 0; i < names.names.size(); ++i) {
      pal_statistics_msgs::msg::Statistic s;
      s.name = names.names[i];
      s.value = values.values[i];
      msg_.statistics.push_back(s);
    }
    last_names_version_ = names.names_version;
  }
}
}  // namespace pal_statistics
