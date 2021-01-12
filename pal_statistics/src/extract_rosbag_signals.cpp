/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <boost/foreach.hpp>
#include <pal_statistics/extract_rosbag_signals.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#define foreach BOOST_FOREACH

namespace pal_statistics {
DataSignals::DataSignals(const std::vector<std::string> &variables_names,
                         const std::string &rosbag_path,
                         const std::string &topic_name) {
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string(topic_name + "/names"));
  topics.push_back(std::string(topic_name + "/values"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::map<std::string, size_t> variables_position_map;
  std::vector<std::string> full_variables_names;

  unsigned int current_names_version = 0;

  foreach (rosbag::MessageInstance const m, view) {
    pal_statistics_msgs::StatisticsNames::ConstPtr statistics_names =
        m.instantiate<pal_statistics_msgs::StatisticsNames>();
    if (statistics_names != NULL) {
      if (statistics_names->names_version != current_names_version) {
        current_names_version = statistics_names->names_version;
        full_variables_names = statistics_names->names;

        for (size_t i = 0; i < variables_names.size(); i++) {
          auto it = std::find(full_variables_names.begin(),
                              full_variables_names.end(), variables_names[i]);

          if (it == full_variables_names.end()) {
            ROS_ERROR_STREAM("Variable " << variables_names[i]
                                         << " not found in the list");
            continue;
          }
          size_t pos = std::distance(full_variables_names.begin(), it);
          variables_position_map[variables_names[i]] = pos;
        }
      }
    }

    pal_statistics_msgs::StatisticsValues::ConstPtr statistics_values =
        m.instantiate<pal_statistics_msgs::StatisticsValues>();
    if (statistics_values != NULL) {
      if (statistics_values->names_version != current_names_version) {
        ROS_WARN_STREAM("Msg names version are different. Skipping data");
      }

      for (auto it = variables_position_map.begin();
           it != variables_position_map.end(); it++) {
        data_[it->first].push_back(
            std::make_pair(statistics_values->header.stamp,
                           statistics_values->values[it->second]));
      }
    }
  }

  bag.close();
}

DataSignals::~DataSignals() {}

std::vector<TimeData>
DataSignals::getDataSignal(const std::string &name) const {
  auto it = data_.find(name);

  if (it == data_.end()) {
    ROS_ERROR_STREAM("Couldn't find signal " << name);
    return std::vector<TimeData>();
  }

  return it->second;
}

std::vector<std::string> DataSignals::getDataSignalNames() const {
  std::vector<std::string> names;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    names.push_back(it->first);
  }
  return names;
}

std::vector<std::vector<TimeData>> DataSignals::getDataValues() const {
  std::vector<std::vector<TimeData>> values;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    values.push_back(it->second);
  }
  return values;
}

size_t DataSignals::getDataValuesMaxSize() const {
  size_t max_size = 0;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    if (max_size < it->second.size())
      max_size = it->second.size();
  }
  return max_size;
}

size_t DataSignals::getDataValuesMinSize() const {
  size_t min_size = std::numeric_limits<size_t>::max();
  for (auto it = data_.begin(); it != data_.end(); it++) {
    if (min_size > it->second.size())
      min_size = it->second.size();
  }
  if(min_size == std::numeric_limits<size_t>::max())
      min_size = 0;

  return min_size;
}

std::vector<TimeData> DataSignals::getDataValue(const ros::Duration &time) const {
  std::vector<TimeData> values_time;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    size_t prev_size = values_time.size();
    for (size_t i = 0; i < it->second.size(); i++) {
      if (time.toSec() <= (it->second[i].first - it->second[0].first).toSec()) {
        values_time.push_back(it->second[i]);
        break;
      }
    }
    if (values_time.size() == prev_size) {
      ROS_WARN_STREAM("Time " << time.toSec() << " is higher than data "
                              << it->first << " time "
                              << it->second.back().first.toSec());
    }
  }
  return values_time;
}

std::map<std::string, int> DataSignals::getIndexMap() const {
  std::map<std::string, int> index_map;
  int i = 0;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    index_map[it->first] = i;
    i++;
  }
  return index_map;
}

std::vector<TimeData> DataSignals::getInstantDataValues(size_t coeff) const {
  std::vector<TimeData> values_coeff;
  for (auto it = data_.begin(); it != data_.end(); it++) {
    if (coeff >= it->second.size()) {
      ROS_WARN_STREAM("Coeff " << coeff << " is higher than data " << it->first
                               << " size " << it->second.size());
      continue;
    }
    values_coeff.push_back(it->second[coeff]);
  }
  return values_coeff;
}
} // namespace pal_statistics
