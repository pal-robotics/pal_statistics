/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/pal_statistics.h>
namespace pal
{

Registration::Registration(const std::string &name, const boost::weak_ptr<StatisticsRegistry> &obj)
  : name_(name), obj_(obj)
{
}

Registration::~Registration()
{
  boost::shared_ptr<StatisticsRegistry> lock = obj_.lock();
  if (lock.get())
    lock->unregisterVariable(name_);
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
  registerVariable("topic_stats." + topic + ".publish_buffer_full_errors", &async_messages_lost_, &internal_stats_raii_);
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

void StatisticsRegistry::registerFunction(const std::string &name,
                                          const boost::function<double()> &funct,
                                          RegistrationsRAII *bookkeeping)
{
  registerInternal(name, VariableHolder(funct), bookkeeping);
}

void StatisticsRegistry::registerVariable(double *variable, const std::string &name,
                                          RegistrationsRAII *bookkeeping)
{
  registerVariable(name, variable, bookkeeping);
}

void StatisticsRegistry::unregisterVariable(const std::string &name, RegistrationsRAII *bookkeeping)
{
  if (bookkeeping)
  {
    bookkeeping->remove(name);
  }

  {
    boost::unique_lock<boost::mutex> data_lock(data_mutex_);
    for (size_t j = msg_.statistics.size(); j > 0; --j)
    {
      size_t i = j -1;
      if (msg_.statistics[i].name == name)
      {
        msg_.statistics.erase(msg_.statistics.begin() + i);
        variables_.erase(variables_.begin() + i);
        //Resize buffer so it contains copies of msg_
        MsgBuffer::VectorType &internal_buffer = msg_buffer_.getBuffer();
        for (size_t v = 0; v < internal_buffer.size(); ++v)
        {
          internal_buffer[v].statistics.erase(internal_buffer[v].statistics.begin() + i);
        }
        
        return;
      }
    }
    ROS_ERROR_STREAM("Tried to unregister variable " << name << " but it is not registered.");
  }
}

void StatisticsRegistry::publish()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  updateMsgUnsafe();

  boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
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

    boost::unique_lock<boost::mutex> data_lock(data_mutex_, boost::adopt_lock);
    if (msg_buffer_.size() == msg_buffer_.capacity())
      async_messages_lost_++;
    // Update stored message with latest data
    updateMsg(msg_buffer_.push_back());    
    
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

void StatisticsRegistry::registerInternal(const std::string &name, VariableHolder &&variable,
                                          RegistrationsRAII *bookkeeping)
{
  {
    boost::unique_lock<boost::mutex> data_lock(data_mutex_);
    variables_.push_back(std::move(variable));
    //Initial value
    pal_statistics_msgs::Statistic s;
    s.name = name;
    s.value = variables_.back().getValue();
    size_t old_capacity = msg_.statistics.capacity();
    msg_.statistics.push_back(s);
    if (msg_.statistics.capacity() > old_capacity)
    {
      //Bulk Resize buffer so it contains copies of msg_
      msg_buffer_.set_capacity(10, msg_);
    }
    else
    {
      MsgBuffer::VectorType &internal_buffer = msg_buffer_.getBuffer();
      for (size_t i = 0; i < internal_buffer.size(); ++i)
      {
        internal_buffer[i].statistics.push_back(s);
      }
    }
  }
  if (bookkeeping)
    bookkeeping->add(boost::make_shared<Registration>(name, weak_from_this()));
}

pal_statistics_msgs::Statistics StatisticsRegistry::createMsg()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  updateMsgUnsafe();
  pal_statistics_msgs::Statistics msg = msg_;
  return msg;
}

void StatisticsRegistry::updateMsgUnsafe()
{
  updateMsg(msg_);
}

void StatisticsRegistry::updateMsg(pal_statistics_msgs::Statistics &msg)
{
  for (size_t i = 0; i < msg.statistics.size(); ++i)
  {
    msg.statistics[i].value = variables_[i].getValue();
  }
  msg.header.stamp = ros::Time::now();
}

void StatisticsRegistry::publisherThreadCycle()
{
  boost::unique_lock<boost::mutex> data_lock(data_mutex_);
  while (!publisher_thread_->interruption_requested() && ros::ok())
  {
    data_ready_cond_.wait(data_lock);
    boost::unique_lock<boost::mutex> pub_lock(pub_mutex_);
    while (msg_buffer_.size() > 0)
    {
      pub_.publish(msg_buffer_.front());
      msg_buffer_.pop_front();
    }
  }
}

void RegistrationsRAII::add(const boost::shared_ptr<Registration> &registration)
{
  boost::unique_lock<boost::mutex> guard(mutex_);
  registrations_.push_back(registration);
}

bool RegistrationsRAII::remove(const std::string &name)
{
  boost::unique_lock<boost::mutex> guard(mutex_);
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it)
  {
    if ((*it)->name_ == name)
    {
      registrations_.erase(it);
      return true;
    }
  }
  return false;
}

void RegistrationsRAII::removeAll()
{
  registrations_.clear();
}
}