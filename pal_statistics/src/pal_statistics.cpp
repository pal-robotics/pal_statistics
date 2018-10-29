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
  pub_ = nh_.advertise<pal_statistics_msgs::Statistics>(topic, 10000);
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
  updateMsg(msg_, true);
  data_lock.unlock(); //msg_ is covered by pub_mutex_

  pub_.publish(msg_);
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
      updateMsg(msg_, true);
      
      data_lock.unlock();
      boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
      pub_.publish(msg_);
      pub_lock.unlock();
      data_lock.lock();
    }
    is_data_ready_ = false;
  }
}
}
