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


#include "pal_statistics/pal_statistics_utils.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pal_statistics/pal_statistics.hpp"

namespace pal_statistics
{

Registration::Registration(
  const std::string & name, IdType id,
  const std::weak_ptr<StatisticsRegistry> & obj)
: name_(name), id_(id), obj_(obj)
{
}

// Registration::Registration(const Registration &&other)
// {
//   name_ = std::move(other.name_);
//   id_ = std::move(other.id_);
//   obj_ = std::move(other.obj_);
// }

Registration::~Registration()
{
  std::shared_ptr<StatisticsRegistry> lock = obj_.lock();
  if (lock.get()) {
    lock->unregisterVariable(id_);
  }
}


std::vector<Registration>::iterator RegistrationsRAII::find(const std::string & name)
{
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it) {
    if ((*it).name_ == name) {
      return it;
    }
  }
  throw std::runtime_error("Unable to find registration with name " + name);
}

std::vector<Registration>::iterator RegistrationsRAII::find(IdType id)
{
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it) {
    if ((*it).id_ == id) {
      return it;
    }
  }
  throw std::runtime_error("Unable to find registration with id " + std::to_string(id));
}


RegistrationsRAII::RegistrationsRAII()
{
}

void RegistrationsRAII::add(Registration && registration)
{
  std::unique_lock<std::mutex> guard(mutex_);
  registrations_.push_back(std::move(registration));
}

bool RegistrationsRAII::remove(const std::string & name)
{
  std::unique_lock<std::mutex> guard(mutex_);
  try {
    registrations_.erase(find(name));
  } catch (...) {
    return false;
  }
  return true;
}


bool RegistrationsRAII::remove(IdType id)
{
  std::unique_lock<std::mutex> guard(mutex_);
  try {
    registrations_.erase(find(id));
  } catch (...) {
    return false;
  }
  return true;
}

void RegistrationsRAII::removeAll()
{
  registrations_.clear();
}

bool RegistrationsRAII::enable(const std::string & name)
{
  Registration & reg = *find(name);
  return reg.obj_.lock()->enable(reg.id_);
}

bool RegistrationsRAII::enable(IdType id)
{
  Registration & reg = *find(id);
  return reg.obj_.lock()->enable(reg.id_);
}

bool RegistrationsRAII::enableAll()
{
  bool result = true;
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it) {
    result &= (*it).obj_.lock()->enable((*it).id_);
  }
  return result;
}

bool RegistrationsRAII::disable(const std::string & name)
{
  Registration & reg = *find(name);
  return reg.obj_.lock()->disable(reg.id_);
}

bool RegistrationsRAII::disable(IdType id)
{
  Registration & reg = *find(id);
  return reg.obj_.lock()->disable(reg.id_);
}

bool RegistrationsRAII::disableAll()
{
  bool result = true;
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it) {
    result |= (*it).obj_.lock()->disable((*it).id_);
  }
  return result;
}
}  // namespace pal_statistics
