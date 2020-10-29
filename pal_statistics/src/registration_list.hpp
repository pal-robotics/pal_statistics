/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#ifndef REGISTRATION_LIST_HPP_
#define REGISTRATION_LIST_HPP_

#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/bimap/set_of.hpp>
#include <pal_statistics/pal_statistics_utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pal_statistics
{

/**
 * @brief The RegistrationList class
 *
 * Not thread safe
 */
class RegistrationList
{
public:
  RegistrationList(
    const std::shared_ptr<rclcpp::Node> & node,
    size_t internal_buffer_capacity = 100);
  int registerVariable(const std::string & name, VariableHolder && holder, bool enabled = true);


  void unregisterVariable(const IdType & id);

  void setEnabled(const IdType & id, bool enabled);


  void unregisterVariable(const std::string & name);

  void doUpdate();

  /**
    @brief fills message with the last captured values.
    */
  void fillMsg(
    pal_statistics_msgs::msg::StatisticsNames & names,
    pal_statistics_msgs::msg::StatisticsValues & value);

  /**
   * @brief smartFillMsg Attempts to minimize the amount of string copies
   * @return true if a smartfill was possible
   *
   * Assumes that msg has already been filled before, and if no variables have been
   * registered/deregistered/enabled/disabled since the last call to this function, will
   * only update the values.
   */
  bool smartFillMsg(
    pal_statistics_msgs::msg::StatisticsNames & names,
    pal_statistics_msgs::msg::StatisticsValues & values);
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
  typedef boost::bimap<boost::bimaps::multiset_of<std::string>,
      boost::bimaps::set_of<IdType>> NameIdBiMap;

  std::shared_ptr<rclcpp::Node> node_;
  NameIdBiMap name_id_;

  size_t buffer_size_;
  std::vector<std::string> names_;
  std::vector<IdType> ids_;
  std::vector<VariableHolder> references_;
  std::vector<bool> enabled_;

  struct NameValues
  {
    explicit NameValues(size_t capacity)
    : names(capacity, IdType(0)), values(capacity, 0.)
    {}

    std::vector<IdType> names;
    std::vector<double> values;
  };

  typedef std::pair<NameValues, rclcpp::Time> LastValuesStamped;
  bool all_enabled_;
  StaticCircularBuffer<LastValuesStamped> last_values_buffer_;

  bool registrations_changed_;
};

}  // namespace pal_statistics
#endif  // REGISTRATION_LIST_HPP_
