/**************************************************************************
**
**  Author: victor
**  Created on: 2018/06/06
**
**  Copyright (c) 2018 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef _PAL_STATISTICS_H_
#define _PAL_STATISTICS_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/thread/lock_guard.hpp>
#include <pal_statistics_msgs/Statistics.h>
namespace pal
{
class StatisticsRegistry;

class Registration
{
public:
  Registration(const std::string &name, const boost::weak_ptr<StatisticsRegistry> &obj);

  ~Registration();

  std::string name_;
  boost::weak_ptr<StatisticsRegistry> obj_;
};


/**
 * @brief The FooSingleton class
 * @warning Functions are not real-time safe unless stated.
 */
class StatisticsRegistry : public boost::enable_shared_from_this<StatisticsRegistry>
{
public:
  typedef std::vector<boost::shared_ptr<Registration> > BookkeepingType;

  StatisticsRegistry(const std::string &topic);
  
  virtual ~StatisticsRegistry();

  void registerVariable(const std::string &name, double *variable,
                        BookkeepingType *bookkeeping = NULL);

  /**
   * Deprecated, required to maintain legacy code
   */
  void registerVariable(double *variable, const std::string &name,
                        BookkeepingType *bookkeeping = NULL);

  void unregisterVariable(const std::string &name, BookkeepingType *bookkeeping = NULL);

  /**
   * @brief publish Reads the values of all registered variables and publishes them to the
   * topic associated to this object.
   * @warning This function may lock if another thread is adding/removing registered
   * variables, use publishAsync if you need RT safety
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

  void startPublishThread();

  pal_statistics_msgs::Statistics createMsg();

private:
  /**
   * @brief fillMsgUnsafe Attempts to create a message without acquiring the mutex
   * Should only be used if the mutex has already been acquired by the thread
   * calling this
   * @param statistics
   * @return Tr
   */
  void fillMsgUnsafe(pal_statistics_msgs::Statistics &msg) const;

  void publisherThreadCycle();

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  boost::shared_mutex variables_mutex_;
  typedef std::vector<std::pair<std::string, double *> > VariablesType;
  boost::shared_ptr<VariablesType> variables_;
  boost::shared_ptr<VariablesType> variables_aux_;

  // Async publisher data
  boost::mutex async_pub_mutex_;
  boost::condition_variable data_ready_cond_;
  boost::shared_ptr<boost::thread> publisher_thread_;
  pal_statistics_msgs::Statistics msg_;
};
}  // namespace pal
#endif