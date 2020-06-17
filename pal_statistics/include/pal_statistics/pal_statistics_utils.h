/**
 *
 * MIT License
 * 
 * Copyright (c) 2019 PAL Robotics S.L.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#ifndef PAL_STATISTICS_UTILS_H
#define PAL_STATISTICS_UTILS_H

#include <boost/lockfree/queue.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/variant.hpp>
#include <atomic>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <pal_statistics_msgs/Statistics.h>
#include <pal_statistics_msgs/StatisticsNames.h>
#include <pal_statistics_msgs/StatisticsValues.h>
#include <pal_statistics/static_circular_buffer.h>
namespace pal_statistics
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
  Registration(const std::string &name, IdType id,
               const boost::weak_ptr<StatisticsRegistry> &obj);
  Registration(Registration &&other) = default;
  Registration &operator=(Registration &&) = default;

  ~Registration();

  std::string name_;
  IdType id_;
  boost::weak_ptr<StatisticsRegistry> obj_;
private:
  // This object should not be copied, because we may unregister variables prematurely
  Registration( const Registration& ) = delete; // non construction-copyable
  Registration& operator=( const Registration& ) = delete; // non copyable
};

/**
 * @brief The RegistrationsRAII class holds handles to registered variables and when it is
 * destroyed, unregisters them automatically.
 */
class RegistrationsRAII
{
public:
  RegistrationsRAII();
  void add(Registration &&registration);
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
  // This object should not be copied, because Registration is not copiable
  RegistrationsRAII( const RegistrationsRAII& ) = delete; // non construction-copyable
  RegistrationsRAII& operator=( const RegistrationsRAII& ) = delete; // non copyable

  std::vector<Registration>::iterator find(const std::string &name);
  std::vector<Registration>::iterator find(IdType id);

  boost::mutex mutex_;
  std::vector<Registration> registrations_;
};


class VariableHolder
{
public:
  VariableHolder()
  {
    // We just implement it because it's needed by the vector resize method, but
    // we never use it to increase the size
    throw std::runtime_error("VariableHolder default constructor should never be called");
  }

  VariableHolder(const double *const pointer) : v_ptr_(pointer)
  {
    v_ptr_ = pointer;
  }

  VariableHolder(const boost::function<double()> &function) : v_ptr_(nullptr), v_func_(function)
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
    v_ptr_ = std::move(other.v_ptr_);
    v_func_ = std::move(other.v_func_);
  }
  ~VariableHolder()
  {
  }

  inline double getValue() const
  {
    if (v_ptr_)
      return *v_ptr_;
    else
      return v_func_();
  }

private:
  const double *v_ptr_;
   boost::function<double()> v_func_;
};


/**
 * @brief The RegistrationList class
 *
 * Not thread safe
 */
class RegistrationList
{
public:
  RegistrationList(size_t internal_buffer_capacity = 100);
  int registerVariable(const std::string &name, VariableHolder &&holder, bool enabled = true);


  void unregisterVariable(const IdType &id);

  void setEnabled(const IdType &id, bool enabled);


  void unregisterVariable(const std::string &name);

  void doUpdate();

  /**
    @brief fills message with the last captured values.
    */
  void fillMsg(pal_statistics_msgs::StatisticsNames &names, pal_statistics_msgs::StatisticsValues &value);

  /**
   * @brief smartFillMsg Attempts to minimize the amount of string copies
   * @return true if a smartfill was possible
   *
   * Assumes that msg has already been filled before, and if no variables have been
   * registered/deregistered/enabled/disabled since the last call to this function, will
   * only update the values.
   */
  bool smartFillMsg(pal_statistics_msgs::StatisticsNames &names, pal_statistics_msgs::StatisticsValues &values);
  /**
   * @return the number of variables registered
   */
  size_t size() const;

  bool hasPendingData() const;


  // How many messages where lost because the buffer was full
  unsigned int overwritten_data_count_;

private:
  void deleteElement(size_t index);
  void registrationsChanged();

  int last_id_;
  unsigned int names_version_;

  // Bidirectional map between names and ids.
  // Can have multiple variables with the same name but different id, but not multiple id
  typedef boost::bimap<boost::bimaps::multiset_of<std::string>, boost::bimaps::set_of<IdType>> NameIdBiMap;

  NameIdBiMap name_id_;

  size_t buffer_size_;
  std::vector<std::string> names_;
  std::vector<IdType> ids_;
  std::vector<VariableHolder> references_;
  std::vector<bool> enabled_;

  struct NameValues
  {
    NameValues(size_t capacity)
      : names(capacity, IdType(0)), values(capacity, 0.)
    {}

    std::vector<IdType> names;
    std::vector<double> values;
  };

  typedef std::pair<NameValues, ros::Time> LastValuesStamped;
  bool all_enabled_;
  StaticCircularBuffer<LastValuesStamped> last_values_buffer_;

  bool registrations_changed_;
};
}

#endif  // PAL_STATISTICS_UTILS_H
