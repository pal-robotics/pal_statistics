/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <gtest/gtest.h>
#include <pal_statistics/extract_rosbag_signals.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace pal_statistics {

TEST(ExtractRosbagSignalsTest, wrong_rosbag) {
  std::vector<std::string> var_names;
  var_names.push_back("var1");
  var_names.push_back("var2");
  var_names.push_back("var3");
  std::string rosbag_path = "wrong_path";
  EXPECT_ANY_THROW(DataSignals data(var_names, rosbag_path));
}

TEST(ExtractRosbagSignalsTest, wrong_variables) {
  std::string path = ros::package::getPath("pal_statistics");
  path += "/test/test.bag";
  std::vector<std::string> var_names;
  DataSignals data_empty(var_names, path);
  EXPECT_TRUE(data_empty.getDataSignalNames().empty());
  EXPECT_TRUE(data_empty.getDataValues().empty());
  EXPECT_EQ(data_empty.getDataValuesMinSize(), 0);
  EXPECT_EQ(data_empty.getDataValuesMaxSize(), 0);
  EXPECT_TRUE(data_empty.getIndexMap().empty());

  var_names.push_back("var1");
  var_names.push_back("var2");

  DataSignals data_wrong_var(var_names, path);
  EXPECT_TRUE(data_wrong_var.getDataSignalNames().empty());
  EXPECT_TRUE(data_wrong_var.getDataValues().empty());
  EXPECT_EQ(data_wrong_var.getDataValuesMinSize(), 0);
  EXPECT_EQ(data_wrong_var.getDataValuesMaxSize(), 0);
  EXPECT_TRUE(data_wrong_var.getIndexMap().empty());
}

TEST(ExtractRosbagSignalsTest, correct_execution) {
  std::string path = ros::package::getPath("pal_statistics");
  path += "/test/test.bag";
  std::vector<std::string> var_names;
  var_names.push_back(
      "/sine_sweep_controller_local_control_actual_effort_leg_left_4_joint");
  var_names.push_back(
      "/sine_sweep_controller_local_control_desired_effort_leg_left_4_joint");
  var_names.push_back("wrong_variable");

  DataSignals data(var_names, path);

  std::vector<std::string> received_var_names = data.getDataSignalNames();
  EXPECT_EQ(received_var_names.size(), 2);
  EXPECT_NE(std::find(received_var_names.begin(), received_var_names.end(),
                      var_names[0]),
            received_var_names.end());
  EXPECT_NE(std::find(received_var_names.begin(), received_var_names.end(),
                      var_names[1]),
            received_var_names.end());

  size_t size_signal = 21915;
  EXPECT_EQ(data.getDataValuesMinSize(), size_signal);
  EXPECT_EQ(data.getDataValuesMaxSize(), size_signal);

  std::map<std::string, int> data_map = data.getIndexMap();
  EXPECT_EQ(data_map.size(), 2);
  EXPECT_EQ(data_map[var_names[0]], 0);
  EXPECT_EQ(data_map[var_names[1]], 1);

  std::vector<std::vector<TimeData>> signals = data.getDataValues();
  EXPECT_EQ(signals.size(), 2);
  EXPECT_EQ(signals[0].size(), size_signal);
  EXPECT_EQ(signals[1].size(), size_signal);

  std::vector<TimeData> signal_0 = data.getDataSignal(var_names[0]);
  std::vector<TimeData> signal_1 = data.getDataSignal(var_names[1]);
  std::vector<TimeData> signal_2 = data.getDataSignal(var_names[2]);

  EXPECT_EQ(signal_0.size(), size_signal);
  EXPECT_EQ(signal_1.size(), size_signal);
  EXPECT_TRUE(signal_2.empty());

  EXPECT_EQ(signal_0, signals[0]);
  EXPECT_EQ(signal_1, signals[1]);

  std::vector<TimeData> instant_signals = data.getDataValue(ros::Duration(8.0));
  std::vector<TimeData> initial_signals = data.getDataValue(ros::Duration(0.0));
  EXPECT_EQ(instant_signals.size(), 2);
  EXPECT_NEAR((instant_signals[0].first - initial_signals[0].first).toSec(),
              8.0, 1.e-4);
  EXPECT_NEAR((instant_signals[1].first - initial_signals[1].first).toSec(),
              8.0, 1.e-4);

  std::vector<TimeData> instant_signals_wrong =
      data.getDataValue(ros::Duration(12.0));
  EXPECT_EQ(instant_signals_wrong.size(), 0);

  std::vector<TimeData> instant_coeff_signals =
      data.getInstantDataValues(size_signal - 1);
  EXPECT_EQ(instant_coeff_signals.size(), 2);

  std::vector<TimeData> instant_coeff_signals_wrong =
      data.getInstantDataValues(size_signal);
  EXPECT_EQ(instant_coeff_signals_wrong.size(), 0);

  std::vector<TimeData> instant_signals_2 = data.getDataValue(ros::Duration(7.7102));
  EXPECT_DOUBLE_EQ(instant_signals_2[0].second, 0);
  EXPECT_DOUBLE_EQ(instant_signals_2[1].second, -12.772910497261488);
}

} // namespace pal_statistics

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_rosbag_signals_test");
  // first nodehandle created of an app must exist until the end of the life of
  // the node
  // If not, you'll have funny stuff such as no logs printed
  ros::NodeHandle nh;
  ros::Time::waitForValid();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
