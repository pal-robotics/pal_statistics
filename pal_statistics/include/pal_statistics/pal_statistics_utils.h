/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#ifndef PAL_STATISTICS_UTILS_H
#define PAL_STATISTICS_UTILS_H

#include <boost/lockfree/queue.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <atomic>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <pal_statistics_msgs/Statistics.h>
namespace pal
{
typedef unsigned int IdType;

/**
 * @brief Simple wrapper around boost lockfree queue to keep track of the reserved memory
 *        Boost's implementation of reserve always increases the capacity by the specified
 * size
 */
template <typename T>
class LockFreeQueue : private boost::lockfree::queue<T>
{
public:
  typedef boost::lockfree::queue<T> BaseType;

  LockFreeQueue() : BaseType(0), reserved_size(0)
  {
  }
  void set_capacity(typename BaseType::size_type n)
  {
    long long missing_size = n - reserved_size;
    if (missing_size > 0)
    {
      BaseType::reserve(missing_size);
      reserved_size += missing_size;
    }
  }

  bool bounded_push(T const &t)
  {
    return BaseType::bounded_push(t);
  }

  template <typename U>
  bool pop(U &ret)
  {
    return BaseType::pop(ret);
  }

private:
  std::atomic<size_t> reserved_size;
};


class StatisticsRegistry;

/**
 * @brief The Registration class is a handle to a registered variable, when out of scope
 * unregisters the variable.
 */
class Registration
{
public:
  Registration(const std::string &name, IdType id, const boost::weak_ptr<StatisticsRegistry> &obj);

  ~Registration();

  std::string name_;
  IdType id_;
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
  bool remove(IdType id);
  void removeAll();

  bool enable(const std::string &name);
  bool enable(IdType id);
  bool enableAll();
  
  bool disable(const std::string &name);
  bool disable(IdType id);
  bool disableAll();
  
private:
  std::vector<boost::shared_ptr<Registration> >::iterator find(const std::string &name);  
  std::vector<boost::shared_ptr<Registration> >::iterator find(IdType id);
  
  boost::mutex mutex_;
  std::vector<boost::shared_ptr<Registration> > registrations_;
};


/// @todo use std::variant
class VariableHolder
{
public:
  VariableHolder()
  {
    // We just implement it because it's needed by the vector resize method, but
    // we never use it to increase the size
    throw std::runtime_error("VariableHolder default constructor should never be called");
  }

  VariableHolder(double *pointer) : is_double_(true), pointer_(pointer)
  {
  }

  VariableHolder(const boost::function<double()> &function)
    : is_double_(false), function_(function)
  {
  }

  VariableHolder(const VariableHolder &other) = delete;
  void operator=(const VariableHolder &other) = delete;

  VariableHolder(const VariableHolder &&other)
  {
    *this = std::move(other);
  }

  void operator=(const VariableHolder &&other)
  {
    is_double_ = std::move(other.is_double_);
    if (other.is_double_)
    {
      pointer_ = std::move(other.pointer_);
    }
    else
    {
      function_ = std::move(other.function_);
    }
  }
  ~VariableHolder()
  {
  }

  double getValue() const
  {
    if (is_double_)
      return *pointer_;
    else
      return function_();
  }

private:
  bool is_double_;

  double *pointer_;
  boost::function<double()> function_;
};


/**
 * @brief The RegistrationList class
 *
 * Not thread safe
 */
class RegistrationList
{
public:
  RegistrationList();
  int registerVariable(const std::string &name, VariableHolder &&holder, bool enabled = true);


  void unregisterVariable(const IdType &id);

  void setEnabled(const IdType &id, bool enabled);


  void unregisterVariable(const std::string &name);

  void doUpdate();

  /**
    @brief fills message with the last captured values.
    */
  void fillMsg(pal_statistics_msgs::Statistics &msg);

  /**
   * @brief smartFillMsg Attempts to minimize the amount of string copies
   *
   * Assumes that msg has already been filled before, and if no variables have been
   * registered/deregistered/enabled/disabled since the last call to this function, will
   * only update the values.
   */
  void smartFillMsg(pal_statistics_msgs::Statistics &msg);
  /**
   * @return the number of variables registered
   */
  size_t size() const;

private:
  void deleteElement(size_t index);

  int last_id_;

  // Bidirectional map between names and ids.
  // Can have multiple variables with the same name but different id, but not multiple id
  typedef boost::bimap<boost::bimaps::multiset_of<std::string>, boost::bimaps::set_of<IdType>> NameIdBiMap;

  NameIdBiMap name_id_;


  std::vector<std::string> names_;
  std::vector<IdType> ids_;
  std::vector<VariableHolder> references_;
  std::vector<bool> enabled_;


  /// @todo apply static circular buffer here
  std::vector<std::pair<IdType, double>> last_values_;

  bool registrations_changed_;

  // Initial value
  //  pal_statistics_msgs::Statistic s;
  //  s.name = name;
  //  s.value = variables_.back().getValue();
  //  size_t old_capacity = msg_.statistics.capacity();
  //  msg_.statistics.push_back(s);
  //  disabled_ids_.set_capacity(msg_.statistics.size());
  //  if (msg_.statistics.capacity() > old_capacity)
  //  {
  //    //Bulk Resize buffer so it contains copies of msg_
  //    msg_buffer_.set_capacity(10, msg_);
  //  }
  //  else
  //  {
  //    MsgBuffer::VectorType &internal_buffer = msg_buffer_.getBuffer();
  //    for (size_t i = 0; i < internal_buffer.size(); ++i)
  //    {
  //      internal_buffer[i].statistics.push_back(s);
  //    }
  //  }
  //}



  //  if (msg_buffer_.size() == msg_buffer_.capacity())
  //    async_messages_lost_++;



  //  while (msg_buffer_.size() > 0)
  //  {
  //    pub_.publish(msg_buffer_.front());
  //    msg_buffer_.pop_front();
  //  }
  
  
//  typedef StaticCircularBuffer<pal_statistics_msgs::Statistics> MsgBuffer;
//  MsgBuffer msg_buffer_; 
};
}

#endif  // PAL_STATISTICS_UTILS_H