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
#include <boost/variant.hpp>
#include <atomic>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <pal_statistics_msgs/Statistics.h>
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

  VariableHolder(const double *const pointer) : variable_(pointer)
  {
  }

  VariableHolder(const boost::function<double()> &function) : variable_(function)
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
    variable_ = std::move(other.variable_);
  }
  ~VariableHolder()
  {
  }

  double getValue() const
  {
    if (variable_.type() == typeid(const double *))
      return *boost::get<const double *>(variable_);
    else
      return boost::get<boost::function<double()>>(variable_)();
  }

private:
  boost::variant<const double *, boost::function<double()>> variable_;
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
  
  bool hasPendingData() const;
  
  
  // How many messages where lost because the buffer was full
  unsigned int overwritten_data_count_;
private:
  void deleteElement(size_t index);
  void registrationsChanged();

  int last_id_;

  // Bidirectional map between names and ids.
  // Can have multiple variables with the same name but different id, but not multiple id
  typedef boost::bimap<boost::bimaps::multiset_of<std::string>, boost::bimaps::set_of<IdType>> NameIdBiMap;

  NameIdBiMap name_id_;

  size_t buffer_size_;
  std::vector<std::string> names_;
  std::vector<IdType> ids_;
  std::vector<VariableHolder> references_;
  std::vector<bool> enabled_;

  typedef std::vector<std::pair<IdType, double>> LastValuesType; 
  StaticCircularBuffer<LastValuesType> last_values_buffer_;

  bool registrations_changed_;
};
}

#endif  // PAL_STATISTICS_UTILS_H