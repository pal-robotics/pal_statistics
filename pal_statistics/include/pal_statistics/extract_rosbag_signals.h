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

/* This class extract the signals from a rosbag */
class DataSignals {
public:

  /* Constructor:
   *  - variables_names: name of the variables to be extracted from the rosbag
   *  - rosbag_path: rosbag location
   *  - topic_name: topic where to extract the variables. By default is /introspection_data. This topic must be splitted as /introspection_data/names, /introspection_data/values
   */
  DataSignals(const std::vector<std::string> &variables_names,
              const std::string &rosbag_path,
              const std::string &topic_name = "/introspection_data");
  ~DataSignals();

  /* Receives the whole signal from a specific variable */
  std::vector<TimeData> getDataSignal(const std::string &name) const;

  /* Receives the names of all the choosen variables in the constructor */
  std::vector<std::string> getDataSignalNames() const;

  /* Receives the signals from the choosen variables in the constructor */
  std::vector<std::vector<TimeData>> getDataValues() const;

  /* From the choosen signals returns the minimum size */
  size_t getDataValuesMinSize() const;

  /* From the choosen signals returns the maximum size */
  size_t getDataValuesMaxSize() const;

  /* Returns a map with the names of the variables and his coefficient in the list */
  std::map<std::string, int> getIndexMap() const;

  /* Returns a screenshot from all the variables at a specific moment of time */
  std::vector<TimeData> getDataValue(const ros::Duration &time) const;

  /* Returns a screenshot from all the variables at a specific coefficient */
  std::vector<TimeData> getInstantDataValues(size_t coeff) const;

private:
  std::map<std::string, std::vector<TimeData>> data_;
};
} // namespace pal_statistics

#endif // EXTRACT_ROSBAG_SIGNALS_H
