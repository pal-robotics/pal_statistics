// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef PAL_STATISTICS__PAL_STATISTICS_UTILS_HPP_
#define PAL_STATISTICS__PAL_STATISTICS_UTILS_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "pal_statistics_msgs/msg/statistics.hpp"
#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include "pal_statistics_msgs/msg/statistics_values.hpp"
#include "pal_statistics/static_circular_buffer.hpp"
#include "rclcpp/rclcpp.hpp"


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
  Registration(
    const std::string & name, IdType id,
    const std::weak_ptr<StatisticsRegistry> & obj);
  Registration(Registration && other) = default;
  Registration & operator=(Registration &&) = default;

  ~Registration();

  std::string name_;
  IdType id_;
  std::weak_ptr<StatisticsRegistry> obj_;

private:
  // This object should not be copied, because we may unregister variables prematurely
  Registration(const Registration &) = delete;  // non construction-copyable
  Registration & operator=(const Registration &) = delete;  // non copyable
};

/**
 * @brief The RegistrationsRAII class holds handles to registered variables and when it is
 * destroyed, unregisters them automatically.
 */
class RegistrationsRAII
{
public:
  RegistrationsRAII();
  void add(Registration && registration);
  bool remove(const std::string & name);
  bool remove(IdType id);
  void removeAll();

  bool enable(const std::string & name);
  bool enable(IdType id);
  bool enableAll();

  bool disable(const std::string & name);
  bool disable(IdType id);
  bool disableAll();

private:
  // This object should not be copied, because Registration is not copiable
  RegistrationsRAII(const RegistrationsRAII &) = delete;  // non construction-copyable
  RegistrationsRAII & operator=(const RegistrationsRAII &) = delete;  // non copyable

  std::vector<Registration>::iterator find(const std::string & name);
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

  explicit VariableHolder(const double * const pointer)
  : v_ptr_(pointer)
  {
    v_ptr_ = pointer;
  }

  explicit VariableHolder(const std::function<double()> & function)
  : v_ptr_(nullptr), v_func_(function)
  {
  }

  VariableHolder(const VariableHolder & other) = delete;
  void operator=(const VariableHolder & other) = delete;

  VariableHolder(const VariableHolder && other)
  {
    *this = std::move(other);
  }

  void operator=(const VariableHolder && other)
  {
    v_ptr_ = std::move(other.v_ptr_);
    v_func_ = std::move(other.v_func_);
  }
  ~VariableHolder()
  {
  }

  inline double getValue() const
  {
    if (v_ptr_) {
      return *v_ptr_;
    } else {
      return v_func_();
    }
  }

private:
  const double * v_ptr_;
  std::function<double()> v_func_;
};

}  // namespace pal_statistics

#endif  // PAL_STATISTICS__PAL_STATISTICS_UTILS_HPP_
