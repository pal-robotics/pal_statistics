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

#include <mutex>
#include <atomic>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <pal_statistics_msgs/msg/statistics.hpp>
#include <pal_statistics_msgs/msg/statistics_names.hpp>
#include <pal_statistics_msgs/msg/statistics_values.hpp>
#include <pal_statistics/static_circular_buffer.h>
namespace pal_statistics
{
typedef unsigned int IdType;

class StatisticsRegistry;

/**
 * @brief The Registration class is a handle to a registered variable, when out of scope
 * unregisters the variable.
 */
class Registration
{
public:
  Registration(const std::string &name, IdType id,
               const std::weak_ptr<StatisticsRegistry> &obj);
  Registration(Registration &&other) = default;
  Registration &operator=(Registration &&) = default;

  ~Registration();

  std::string name_;
  IdType id_;
  std::weak_ptr<StatisticsRegistry> obj_;
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

  std::mutex mutex_;
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

  VariableHolder(const std::function<double()> &function) : v_ptr_(nullptr), v_func_(function)
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
   std::function<double()> v_func_;
};

}

#endif  // PAL_STATISTICS_UTILS_H
