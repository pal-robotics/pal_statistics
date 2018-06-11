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
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/thread.hpp>
#include <pal_statistics_msgs/Statistics.h>
namespace pal
{
class StatisticsRegistry;

/**
 * @brief The Registration class is a handle to a registered variable, when out of scope
 * unregisters the variable.
 */
class Registration
{
public:
  Registration(const std::string &name, const boost::weak_ptr<StatisticsRegistry> &obj);

  ~Registration();

  std::string name_;
  boost::weak_ptr<StatisticsRegistry> obj_;
};

/**
 * @brief The RegistrationsRAII class holds handles to registered variables and when it is
 * destroyed, unregisters them automatically.
 */
class RegistrationsRAII
{
public:
  void add(const boost::shared_ptr<Registration> &registration);
  bool remove(const std::string &name);

private:
  boost::mutex mutex_;
  std::vector<boost::shared_ptr<Registration> > registrations_;
};

/**
 * @brief The StatisticsRegistry class reads the value of registered variables and
 * publishes them on the specified topic.
 * @warning Functions are not real-time safe unless stated.
 */
class StatisticsRegistry : public boost::enable_shared_from_this<StatisticsRegistry>
{
public:
  StatisticsRegistry(const std::string &topic);

  virtual ~StatisticsRegistry();

  /**
   * @brief registerVariable adds the variable so its value is later published with the
   * specified name
   * @param bookkeeping Optional, if specified adds a handle to this variable
   * registration, so registration is done when this object goes out of scope.
   */
  void registerVariable(const std::string &name, double *variable,
                        RegistrationsRAII *bookkeeping = NULL);

  /**
   * Deprecated, required to maintain legacy code
   */
  void registerVariable(double *variable, const std::string &name,
                        RegistrationsRAII *bookkeeping = NULL);

  void unregisterVariable(const std::string &name, RegistrationsRAII *bookkeeping = NULL);

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
  pal_statistics_msgs::Statistics createMsg();

private:
  /**
   * @brief updateMsgUnsafe Updates the internal message variable without acquiring the
   * mutex Should only be used if the mutex has already been acquired by the thread
   * calling this
   */

  void updateMsgUnsafe();

  void publisherThreadCycle();

  ros::NodeHandle nh_;

  boost::mutex data_mutex_;
  typedef std::vector<double *> VariablesType;
  VariablesType variables_;
  pal_statistics_msgs::Statistics msg_;

  // To avoid deadlocks, should always be acquired after data_mutex_
  boost::mutex pub_mutex_;
  ros::Publisher pub_;
  boost::condition_variable data_ready_cond_;
  boost::shared_ptr<boost::thread> publisher_thread_;
  double publish_async_attempts_;
  double publish_async_failures_;
};
}  // namespace pal
#endif
