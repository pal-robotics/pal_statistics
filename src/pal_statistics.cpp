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
  pub_ = nh_.advertise<pal_statistics_msgs::Statistics>(topic, 10);
}

void StatisticsRegistry::registerVariable(const std::string &name, double *variable,
                                    StatisticsRegistry::BookkeepingType *bookkeeping)
{
  {
    boost::unique_lock<boost::shared_mutex> write_lock(variables_mutex_);
    variables_.push_back(std::make_pair(name, variable));
    boost::lock_guard<boost::mutex> lock_async(async_pub_mutex_);
    // reserve enough space for all variables
    foo_.statistics.reserve(variables_.size());
  }
  if (bookkeeping)
    bookkeeping->push_back(boost::make_shared<Registration>(name, weak_from_this()));
}

void StatisticsRegistry::registerVariable(double *variable, const std::string &name, 
                                    StatisticsRegistry::BookkeepingType *bookkeeping)
{
  registerVariable(name, variable, bookkeeping);
}

void StatisticsRegistry::unregisterVariable(const std::string &name,
                                      StatisticsRegistry::BookkeepingType *bookkeeping)
{
  if (bookkeeping)
  {
    for (BookkeepingType::iterator it = bookkeeping->begin(); it != bookkeeping->end(); ++it)
    {
      if ((*it)->name_ == name)
      {
        bookkeeping->erase(it);
        break;
      }
    }
  }
  boost::unique_lock<boost::shared_mutex> write_lock(variables_mutex_);
  for (VariablesType::iterator it = variables_.begin(); it != variables_.end(); ++it)
  {
    if (it->first == name)
    {
      variables_.erase(it);
      return;
    }
  }
}

void StatisticsRegistry::publish()
{
  pal_statistics_msgs::Statistics msg = createMsg();
  pub_.publish(msg);
}

bool StatisticsRegistry::publishAsync()
{
  if (!publisher_thread_.get())
  {
    ROS_WARN("Called publishAsync but publisher thread has not been started, starting it, this is not RT safe");
    startPublishThread();
  }
  if (variables_mutex_.try_lock_shared())
  {
    boost::shared_lock<boost::shared_mutex> read_lock(variables_mutex_, boost::adopt_lock);
    boost::lock_guard<boost::mutex> lock_async(async_pub_mutex_);
    // Update stored message with latest data
    fillMsgUnsafe(foo_);
    data_ready_cond_.notify_one();
    return true;
  }
  else
  {
    ROS_DEBUG("Missed publishRT opportunity because lock could not be acquired.");
    return false;
  }
}

void StatisticsRegistry::startPublishThread()
{
  publisher_thread_.reset(new boost::thread(&StatisticsRegistry::publisherThreadCycle, this));
}

pal_statistics_msgs::Statistics StatisticsRegistry::createMsg()
{
  boost::shared_lock<boost::shared_mutex> read_lock(variables_mutex_);
  pal_statistics_msgs::Statistics msg;
  fillMsgUnsafe(msg);
  return msg;
}

void StatisticsRegistry::fillMsgUnsafe(pal_statistics_msgs::Statistics &msg) const
{
  
  msg.statistics.resize(0);  // delete all elements
  for (size_t i = 0; i < variables_.size(); ++i)
  {
    pal_statistics_msgs::Statistic s;
    s.name = variables_[i].first;
    s.value = *variables_[i].second;
    msg.statistics.push_back(s);
  }
  msg.header.stamp = ros::Time::now();
}

void StatisticsRegistry::publisherThreadCycle()
{
  boost::unique_lock<boost::mutex> lock_async(async_pub_mutex_);
  while (true)
  {
    data_ready_cond_.wait(lock_async);
    if (pub_.getNumSubscribers() > 0)
    {
      pub_.publish(foo_);
    }
  }
}
}