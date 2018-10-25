/*
  @file
  
  @author victor
  
  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <pal_statistics/pal_statistics_utils.h>
#include <pal_statistics/pal_statistics.h>

namespace pal_statistics
{

Registration::Registration(const std::string &name, IdType id, const boost::weak_ptr<StatisticsRegistry> &obj)
  : name_(name), id_(id), obj_(obj)
{
}

//Registration::Registration(const Registration &&other)
//{
//  name_ = std::move(other.name_);
//  id_ = std::move(other.id_);
//  obj_ = std::move(other.obj_);
//}

Registration::~Registration()
{
  boost::shared_ptr<StatisticsRegistry> lock = obj_.lock();
  if (lock.get())
    lock->unregisterVariable(id_);
}

RegistrationList::RegistrationList(size_t internal_buffer_capacity)
  : last_id_(0), registrations_changed_(true), buffer_size_(internal_buffer_capacity)
{
  overwritten_data_count_ = 0;
}

void RegistrationList::unregisterVariable(const IdType &id)
{
  for (size_t i = 0; i < ids_.size(); ++i)
  {
    if (ids_[i] == id)
    {
      deleteElement(i);
      break;
    }
  }
}

void RegistrationList::setEnabled(const IdType &id, bool enabled)
{
  registrationsChanged();
  for (size_t i = 0; i < ids_.size(); ++i)
  {
    if (ids_[i] == id)
    {
      enabled_[i] = enabled;
      break;
    }
  }
}

void RegistrationList::unregisterVariable(const std::string &name)
{
  size_t count = name_id_.left.count(name);
  if (count > 1)
  {
    ROS_ERROR_STREAM(
          "You asked to unregister "
          << name
          << " but there are multiple variables registered with that name. This can have undefined behaviour, unregistering all");
  }
  if (count == 0)
  {
    ROS_ERROR_STREAM("Tried to unregister variable " << name << " but it is not registered.");
    return;
  }
  auto it = name_id_.left.find(name);
  while (it != name_id_.left.end())
  {
    unregisterVariable(it->second);
    it = name_id_.left.find(name);
  }
}

void RegistrationList::doUpdate()
{
  if (last_values_buffer_.size() == last_values_buffer_.capacity())
    overwritten_data_count_++;
  
  auto &last_values = last_values_buffer_.push_back();
  last_values.clear();
  assert(last_values.capacity() >= ids_.size());
  for (size_t i = 0; i < ids_.size(); ++i)
  {
    if (enabled_[i])
    {
      // Should never allocate memory because its capacity is able to hold all
      // variables
      last_values.emplace_back(ids_[i], references_[i].getValue());
    }
  }
}

void RegistrationList::fillMsg(pal_statistics_msgs::Statistics &msg)
{
  msg.statistics.clear();
  for (auto it : last_values_buffer_.front())
  {
    IdType id = it.first;
    pal_statistics_msgs::Statistic s;
    assert(name_id_.right.find(id) != name_id_.right.end());
    s.name = name_id_.right.find(id)->second;
    s.value = it.second;
    msg.statistics.push_back(s);
  }
  last_values_buffer_.pop_front();
}

void RegistrationList::smartFillMsg(pal_statistics_msgs::Statistics &msg)
{
  if (msg.statistics.empty() || registrations_changed_)
  {
    fillMsg(msg);
    registrations_changed_ = false;
    return;
  }
  
  assert(msg.statistics.size() == last_values_buffer_.front().size());
  for (size_t i = 0; i < msg.statistics.size(); ++i)
  {
    msg.statistics[i].value = last_values_buffer_.front()[i].second;
  }
  last_values_buffer_.pop_front();
}

size_t RegistrationList::size() const
{
  return ids_.size();
}

bool RegistrationList::hasPendingData() const
{
  return last_values_buffer_.size() > 0;
}

void RegistrationList::deleteElement(size_t index)
{
  IdType id = ids_[index];
  if (name_id_.right.count(id) == 0)
    ROS_ERROR_STREAM("Didn't find index " << index << " in <name, index> multimap");
  
  name_id_.right.erase(id);
  
  // Faster way of removing an object of a vector if we don't care about the
  // internal ordering. We just care that they all have the same order
  std::swap(names_[index], names_.back());
  names_.resize(names_.size() - 1);
  std::swap(ids_[index], ids_.back());
  ids_.resize(ids_.size() - 1);
  std::swap(references_[index], references_.back());
  references_.resize(references_.size() - 1);
  std::swap(enabled_[index], enabled_.back());
  enabled_.resize(enabled_.size() - 1);
  
  registrationsChanged();
}

void RegistrationList::registrationsChanged()
{
  registrations_changed_ = true;
  last_values_buffer_.clear();
}

int RegistrationList::registerVariable(const std::string &name, VariableHolder &&holder, bool enabled)
{
  registrationsChanged();
  
  bool needs_more_capacity = (names_.size() == names_.capacity());
  int id = last_id_++;
  name_id_.left.insert(std::make_pair(name, id));
  names_.push_back(name);
  ids_.push_back(id);
  references_.push_back(std::move(holder));
  enabled_.push_back(enabled);
  // reserve memory for values
  if (needs_more_capacity )
  {
    // Reset last_values_buffer_ size to be able to contain buffer_size_ copies 
    // of the last values vector with the same capacity as the names vector
    // But the buffer's number of elements is set to 0
    last_values_buffer_.set_capacity(buffer_size_, 
                                     LastValuesType(names_.capacity(), std::make_pair(0, 0.)));
  }
  return id;
}
std::vector<Registration>::iterator RegistrationsRAII::find(const std::string &name)
{
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it)
  {
    if ((*it).name_ == name)
    {
      return it;
    }
  }
  throw std::runtime_error("Unable to find registration with name " + name);
}

std::vector<Registration>::iterator RegistrationsRAII::find(IdType id)
{
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it)
  {
    if ((*it).id_ == id)
    {
      return it;
    }
  }
  throw std::runtime_error("Unable to find registration with id " + std::to_string(id));
}


RegistrationsRAII::RegistrationsRAII()
{
  
}

void RegistrationsRAII::add(Registration &&registration)
{
  boost::unique_lock<boost::mutex> guard(mutex_);
  registrations_.push_back(std::move(registration));
}

bool RegistrationsRAII::remove(const std::string &name)
{
  boost::unique_lock<boost::mutex> guard(mutex_);
  try
  {
    registrations_.erase(find(name));
  }
  catch (...)
  {
    return false;
  }
  return true;
}


bool RegistrationsRAII::remove(IdType id)
{
  boost::unique_lock<boost::mutex> guard(mutex_);
  try
  {
    registrations_.erase(find(id));
  }
  catch (...)
  {
    return false;
  }
  return true;
}

void RegistrationsRAII::removeAll()
{
  registrations_.clear();
}

bool RegistrationsRAII::enable(const std::string &name)
{
  Registration&reg = *find(name);
  return reg.obj_.lock()->enable(reg.id_);
}

bool RegistrationsRAII::enable(IdType id)
{
  Registration&reg = *find(id);
  return reg.obj_.lock()->enable(reg.id_);  
}

bool RegistrationsRAII::enableAll()
{
  bool result = true;
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it)
  {
    result &= (*it).obj_.lock()->enable((*it).id_);
  }
  return result;
}

bool RegistrationsRAII::disable(const std::string &name)
{
  Registration&reg = *find(name);
  return reg.obj_.lock()->disable(reg.id_);
}

bool RegistrationsRAII::disable(IdType id)
{
  Registration&reg = *find(id);
  return reg.obj_.lock()->disable(reg.id_);  
}

bool RegistrationsRAII::disableAll()
{  
  bool result = true;
  for (auto it = registrations_.begin(); it != registrations_.end(); ++it)
  {
    result |= (*it).obj_.lock()->disable((*it).id_);
  }
  return result;
}
}