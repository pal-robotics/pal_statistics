/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/pal_statistics.h>
#include <pal_statistics/registration_utils.h>
namespace pal_statistics
{

StatisticsRegistry::StatisticsRegistry(const std::string &topic)
{
  pub_ = nh_.advertise<pal_statistics_msgs::Statistics>(topic + "/full", 10000);
  pub_names_ = nh_.advertise<pal_statistics_msgs::StatisticsNames>(topic + "/names", 10000, true);
  pub_values_ = nh_.advertise<pal_statistics_msgs::StatisticsValues>(topic + "/values", 10000);
  publish_async_attempts_ = 0;
  publish_async_failures_ = 0;
  last_async_pub_duration_ = 0.0;
  is_data_ready_ = false;

  customRegister(*this, "topic_stats." + topic + ".publish_async_attempts", &publish_async_attempts_, &internal_stats_raii_);
  customRegister(*this, "topic_stats." + topic + ".publish_async_failures", &publish_async_failures_, &internal_stats_raii_);
  customRegister(*this, "topic_stats." + topic + ".publish_buffer_full_errors", &registration_list_.overwritten_data_count_, &internal_stats_raii_);
  customRegister(*this, "topic_stats." + topic + ".last_async_pub_duration", &last_async_pub_duration_, &internal_stats_raii_);
}

StatisticsRegistry::~StatisticsRegistry()
{
  is_data_ready_ = true; //To let the thread exit nicely

  if (publisher_thread_)
  {
    publisher_thread_->interrupt();
    publisher_thread_->join();
  }
  ROS_INFO_STREAM("Async messages lost " << registration_list_.overwritten_data_count_);
  ROS_INFO_STREAM("publish_async_failures_ " << publish_async_failures_);
}

IdType StatisticsRegistry::registerFunction(const std::string &name,
                                          const boost::function<double()> &funct,
                                          RegistrationsRAII *bookkeeping, bool enabled)
{
  return registerInternal(name, VariableHolder(funct), bookkeeping, enabled);
}

void StatisticsRegistry::unregisterVariable(IdType id, RegistrationsRAII *bookkeeping)
{
  if (bookkeeping)
  {
    bookkeeping->remove(id);
  }

  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  registration_list_.unregisterVariable(id);
}

void StatisticsRegistry::unregisterVariable(const std::string &name, RegistrationsRAII *bookkeeping)
{
  if (bookkeeping)
  {
    bookkeeping->remove(name);
  }

  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  registration_list_.unregisterVariable(name);
}


void StatisticsRegistry::publish()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  handlePendingDisables(data_lock);
  registration_list_.doUpdate();

  boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
  bool minor_changes = updateMsg(names_msg_, values_msg_, true);
  data_lock.unlock(); //msg_ is covered by pub_mutex_
  doPublish(!minor_changes);
}

bool StatisticsRegistry::publishAsync()
{
  double begin = ros::SteadyTime::now().toSec();
  publish_async_attempts_++;
  if (data_mutex_.try_lock())
  {
    if (!publisher_thread_.get())
    {
      ROS_WARN_STREAM_ONCE("Called publishAsync but publisher thread has not been started, THIS IS NOT RT safe. You should start it yourself.");
      startPublishThreadImpl();
    }

    {
      boost::unique_lock<boost::mutex> data_lock(data_mutex_, boost::adopt_lock);
      handlePendingDisables(data_lock);

      registration_list_.doUpdate();
    }
    is_data_ready_ = true;

    last_async_pub_duration_ = ros::SteadyTime::now().toSec() - begin;
    return true;
  }
  publish_async_failures_++;
  // Commented for RT safety
  // ROS_DEBUG("Missed publishRT opportunity because lock could not be acquired.");
  return false;
}

void StatisticsRegistry::startPublishThread()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  startPublishThreadImpl();
}
void StatisticsRegistry::startPublishThreadImpl()
{
  publisher_thread_.reset(new boost::thread(&StatisticsRegistry::publisherThreadCycle, this));
}

IdType StatisticsRegistry::registerInternal(const std::string &name, VariableHolder &&variable,
                                          RegistrationsRAII *bookkeeping, bool enabled)
{
  IdType id;
  {
    boost::unique_lock<boost::mutex> data_lock(data_mutex_);
    id = registration_list_.registerVariable(name, std::move(variable), enabled);
    enabled_ids_.set_capacity(registration_list_.size());
  }

  if (bookkeeping)
    bookkeeping->add(Registration(name, id, weak_from_this()));
  return id;
}

bool StatisticsRegistry::setEnabledmpl(const IdType &id, bool enabled)
{
  EnabledId aux;
  aux.enabled = enabled;
  aux.id = id;

  return enabled_ids_.bounded_push(aux);
}

void StatisticsRegistry::handlePendingDisables(const boost::unique_lock<boost::mutex> &data_lock)
{
  if (!data_lock.owns_lock() || data_lock.mutex() != &data_mutex_)
  {
    throw ros::Exception("Called handlePendingDisables without proper lock");
  }

  EnabledId elem;
  while (enabled_ids_.pop(elem))
  {
    registration_list_.setEnabled(elem.id, elem.enabled);
  }
}

void StatisticsRegistry::doPublish(bool publish_names_msg)
{
  if (pub_.getNumSubscribers() > 0)
  {
    generated_statistics_.update(names_msg_, values_msg_);
    pub_.publish(generated_statistics_.msg_);
  }

  // We don't check subscribers here, because this topic is latched and we
  // always want the latest version published
  if (publish_names_msg) //only publish strings if changed
  {
    pub_names_.publish(names_msg_);
  }
  if (pub_values_.getNumSubscribers() > 0 ) //only publish strings if changed
  {
    pub_values_.publish(values_msg_);
  }
}

pal_statistics_msgs::Statistics StatisticsRegistry::createMsg()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  handlePendingDisables(data_lock);
  registration_list_.doUpdate();
  GeneratedStatistics gen_sts;
  pal_statistics_msgs::StatisticsNames names;
  pal_statistics_msgs::StatisticsValues values;

  updateMsg(names, values, false);
  gen_sts.update(names, values);
  return gen_sts.msg_;
}

bool StatisticsRegistry::enable(const IdType &id)
{
  return setEnabledmpl(id, true);
}

bool StatisticsRegistry::disable(const IdType &id)
{
  return setEnabledmpl(id, false);
}

bool StatisticsRegistry::updateMsg(pal_statistics_msgs::StatisticsNames &names,
                                   pal_statistics_msgs::StatisticsValues &values,
                                   bool smart_fill)
{
  if (smart_fill)
    return registration_list_.smartFillMsg(names, values);
  else
  {
    registration_list_.fillMsg(names, values);
    return false;
  }
}

void StatisticsRegistry::publisherThreadCycle()
{
  //wait until the variable is set
  while (!publisher_thread_.get())
    ros::WallDuration(5e-4).sleep();


  while (ros::ok() && !publisher_thread_->interruption_requested())
  {
    while (!is_data_ready_ && !publisher_thread_->interruption_requested())
      ros::WallDuration(5e-4).sleep();

    boost::unique_lock<boost::mutex> data_lock(data_mutex_);

    while (registration_list_.hasPendingData())
    {
      bool minor_changes = updateMsg(names_msg_, values_msg_, true);

      boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
      data_lock.unlock();
      doPublish(!minor_changes);
      pub_lock.unlock();
      data_lock.lock();
    }
    is_data_ready_ = false;
  }
}

void StatisticsRegistry::GeneratedStatistics::update(
    const pal_statistics_msgs::StatisticsNames &names,
    const pal_statistics_msgs::StatisticsValues &values)
{
  msg_.header = values.header;
  if (last_names_version_ == names.names_version && !msg_.statistics.empty())
  {
    // only need to update the values
    for (size_t i = 0; i < values.values.size(); ++i)
    {
      msg_.statistics[i].value = values.values[i];
    }
  }
  else
  {
    msg_.statistics.clear();
    for (size_t i = 0; i < names.names.size(); ++i)
    {
      pal_statistics_msgs::Statistic s;
      s.name = names.names[i];
      s.value = values.values[i];
      msg_.statistics.push_back(s);
    }
    last_names_version_ = names.names_version;
  }
}
}
