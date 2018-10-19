/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/pal_statistics.h>
namespace pal
{

Registration::Registration(const std::string &name, IdType id, const boost::weak_ptr<StatisticsRegistry> &obj)
  : name_(name), id_(id), obj_(obj)
{
}

Registration::~Registration()
{
  boost::shared_ptr<StatisticsRegistry> lock = obj_.lock();
  if (lock.get())
    lock->unregisterVariable(id_);
}

StatisticsRegistry::StatisticsRegistry(const std::string &topic)
{
  pub_ = nh_.advertise<pal_statistics_msgs::Statistics>(topic, 10000);
  publish_async_attempts_ = 0;
  publish_async_failures_ = 0;
  async_messages_lost_ = 0;
  last_async_pub_duration_ = 0.0;
  registerVariable("topic_stats." + topic + ".publish_async_attempts", &publish_async_attempts_, &internal_stats_raii_);
  registerVariable("topic_stats." + topic + ".publish_async_failures", &publish_async_failures_, &internal_stats_raii_);
//  registerVariable("topic_stats." + topic + ".publish_buffer_full_errors", &async_messages_lost_, &internal_stats_raii_);
  registerVariable("topic_stats." + topic + ".publish_buffer_full_errors", &registration_list_.overwritten_data_count_, &internal_stats_raii_);
  registerVariable("topic_stats." + topic + ".last_async_pub_duration", &last_async_pub_duration_, &internal_stats_raii_);
}

StatisticsRegistry::~StatisticsRegistry()
{
  data_ready_cond_.notify_all();

  if (publisher_thread_)
  {
    publisher_thread_->interrupt();
    publisher_thread_->join();
  }
  ROS_INFO_STREAM("Async messages lost " << async_messages_lost_);
  ROS_INFO_STREAM("publish_async_failures_ " << publish_async_failures_);
}

IdType StatisticsRegistry::registerFunction(const std::string &name,
                                          const boost::function<double()> &funct,
                                          RegistrationsRAII *bookkeeping, bool enabled)
{
  return registerInternal(name, VariableHolder(funct), bookkeeping, enabled);
}

IdType StatisticsRegistry::registerVariable(double *variable, const std::string &name,
                                          RegistrationsRAII *bookkeeping, bool enabled)
{
  return registerVariable(name, variable, bookkeeping, enabled);
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
  
  boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
  updateMsg(msg_, true);
  data_lock.unlock(); //msg_ is covered by pub_mutex_

  pub_.publish(msg_);
}

bool StatisticsRegistry::publishAsync()
{
  double begin = ros::Time::now().toSec();
  publish_async_attempts_++;
  if (data_mutex_.try_lock())
  {
    if (!publisher_thread_.get())
    {
      ROS_WARN_STREAM_ONCE("Called publishAsync but publisher thread has not been started, nothing will be published");
    }

    {
      boost::unique_lock<boost::mutex> data_lock(data_mutex_, boost::adopt_lock);
      handlePendingDisables(data_lock);
      
      registration_list_.doUpdate();    
    }
    data_ready_cond_.notify_one();

    last_async_pub_duration_ = ros::Time::now().toSec() - begin;
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
    bookkeeping->add(boost::make_shared<Registration>(name, id, weak_from_this()));
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

pal_statistics_msgs::Statistics StatisticsRegistry::createMsg()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  handlePendingDisables(data_lock);
  registration_list_.doUpdate();
  pal_statistics_msgs::Statistics msg;
  updateMsg(msg, false);
  return msg;
}

bool StatisticsRegistry::enable(const IdType &id)
{
  return setEnabledmpl(id, true);  
}

bool StatisticsRegistry::disable(const IdType &id)
{
  return setEnabledmpl(id, false);  
}

void StatisticsRegistry::updateMsg(pal_statistics_msgs::Statistics &msg, bool smart_fill)
{
  if (smart_fill)
    registration_list_.smartFillMsg(msg);
  else
    registration_list_.fillMsg(msg);
  msg.header.stamp = ros::Time::now();
}

void StatisticsRegistry::publisherThreadCycle()
{
  while (!publisher_thread_->interruption_requested() && ros::ok())
  {
    boost::unique_lock<boost::mutex> data_lock(data_mutex_);
    if (!registration_list_.hasPendingData())
      data_ready_cond_.wait(data_lock);

    boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
    updateMsg(msg_, true);

    data_lock.unlock();  // Mutex is not needed for publishing
    pub_.publish(msg_);
  }
}
}
