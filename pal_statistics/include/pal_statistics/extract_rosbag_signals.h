/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef EXTRACT_ROSBAG_SIGNALS_H
#define EXTRACT_ROSBAG_SIGNALS_H

#include <pal_statistics/pal_statistics_utils.h>

namespace pal_statistics {
typedef std::pair<ros::Time, double> TimeData;

class DataSignals {
public:
  DataSignals(const std::vector<std::string> &variables_names,
              const std::string &rosbag_path,
              const std::string &topic_name = "/introspection_data");
  ~DataSignals();

  std::vector<TimeData> getDataSignal(const std::string &name) const;

  std::vector<std::string> getDataSignalNames() const;

  std::vector<std::vector<TimeData>> getDataValues() const;

  size_t getDataValuesMinSize() const;

  size_t getDataValuesMaxSize() const;

  std::map<std::string, int> getIndexMap() const;

  std::vector<TimeData> getDataValue(const ros::Time &time) const;

  std::vector<TimeData> getInstantDataValues(size_t coeff) const;

private:
  std::map<std::string, std::vector<TimeData>> data_;
};
} // namespace pal_statistics

#endif // EXTRACT_ROSBAG_SIGNALS_H
