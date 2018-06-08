/**************************************************************************
**
**  File: gtest_smach_c
**
**  Author: victor
**  Created on: 2017/4/13
**
**  Copyright (c) 2017 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#include <pal_statistics/pal_statistics_macros.h>
#include <pal_statistics/pal_statistics.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

using ::testing::UnorderedElementsAre;
namespace pal
{
class PalStatisticsTest : public ::testing::Test
{
public:
  PalStatisticsTest() : spinner_(1, &queue_)
  {
    var1_ = 0.0;
    var2_ = 0.5;

    nh_.setCallbackQueue(&queue_);
    sub_ = nh_.subscribe(DEFAULT_STATISTICS_TOPIC, 1, &PalStatisticsTest::topicCb, this);
    spinner_.start();
  }

  void topicCb(const pal_statistics_msgs::StatisticsConstPtr &msg)
  {
    last_msg_ = msg;
  }

protected:
  double var1_;
  double var2_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_;
  pal_statistics_msgs::StatisticsConstPtr last_msg_;
};


std::vector<std::string> getVariables(const pal_statistics_msgs::Statistics &msg)
{
  std::vector<std::string> v;
  for (auto s : msg.statistics)
  {
    v.push_back(s.name);
  }
  return v;
}

std::map<std::string, double> getVariableAndValues(const pal_statistics_msgs::Statistics &msg)
{
  std::map<std::string, double> m;
  for (auto s : msg.statistics)
  {
    m[s.name] = s.value;
  }
  return m;
}
TEST_F(PalStatisticsTest, checkValues)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);

  registry->registerVariable("var1", &var1_);
  registry->registerVariable("var2", &var2_);

  var1_ = 1.0;
  var2_ = 2.0;
  pal_statistics_msgs::Statistics msg = registry->createMsg();

  EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
  auto s = getVariableAndValues(msg);
  EXPECT_EQ(var1_, s["var1"]);
  EXPECT_EQ(var2_, s["var2"]);

  var1_ = 100.0;
  var2_ = -100.0;
  msg = registry->createMsg();
  s = getVariableAndValues(msg);
  EXPECT_EQ(var1_, s["var1"]);
  EXPECT_EQ(var2_, s["var2"]);
}

TEST_F(PalStatisticsTest, manualRegistration)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);

  registry->registerVariable("var1", &var1_);
  registry->registerVariable("var2", &var2_);

  pal_statistics_msgs::Statistics msg = registry->createMsg();

  EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
  EXPECT_THAT(getVariables(msg), UnorderedElementsAre("var1", "var2"));

  registry->unregisterVariable("var1");
  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg), UnorderedElementsAre("var2"));
}


TEST_F(PalStatisticsTest, automaticRegistration)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  pal_statistics_msgs::Statistics msg;
  {
    StatisticsRegistry::BookkeepingType bookkeeping;

    registry->registerVariable("var1", &var1_, &bookkeeping);
    registry->registerVariable("var2", &var2_, &bookkeeping);

    msg = registry->createMsg();

    EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
    EXPECT_THAT(getVariables(msg), UnorderedElementsAre("var1", "var2"));
  }

  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg), UnorderedElementsAre());
}


TEST_F(PalStatisticsTest, automaticRegistrationDestruction)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  {
    StatisticsRegistry::BookkeepingType bookkeeping;

    registry->registerVariable("var1", &var1_, &bookkeeping);
    registry->registerVariable("var2", &var2_, &bookkeeping);

    registry->createMsg();

    // Delete the main class, the bookkeeper shouldn't crash on destruction
    registry.reset();
  }
}

TEST_F(PalStatisticsTest, asyncPublisher)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  {
    StatisticsRegistry::BookkeepingType bookkeeping;

    registry->startPublishThread();

    // Time for publisher to connect to subscriber
    ros::Duration(0.5).sleep();
    registry->registerVariable("var1", &var1_, &bookkeeping);
    registry->registerVariable("var2", &var2_, &bookkeeping);

    registry->publishAsync();
    ros::Duration(0.1).sleep();
    ASSERT_TRUE(last_msg_.get());

    auto s = getVariableAndValues(*last_msg_);
    EXPECT_EQ(var1_, s["var1"]);
    EXPECT_EQ(var2_, s["var2"]);

    last_msg_.reset();
    ros::Duration(1.0).sleep();
    EXPECT_FALSE(last_msg_.get())
        << " Data shouldn't have been published because there were no calls to publishAsync";

    var1_ = 2.0;
    var2_ = 3.0;
    registry->publishAsync();
    ros::Duration(0.1).sleep();
    ASSERT_TRUE(last_msg_.get());

    s = getVariableAndValues(*last_msg_);
    EXPECT_EQ(var1_, s["var1"]);
    EXPECT_EQ(var2_, s["var2"]);

    last_msg_.reset();
  }
  registry->publishAsync();
  ros::Duration(0.1).sleep();
  ASSERT_TRUE(last_msg_.get());
  EXPECT_EQ(0, last_msg_->statistics.size());
}

TEST_F(PalStatisticsTest, macroTest)
{
  {
    StatisticsRegistry::BookkeepingType bookkeeping;
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1", &var1_, NULL);
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1_bk", &var1_, &bookkeeping);
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2", &var2_, NULL);
    START_PUBLISH_THREAD(DEFAULT_STATISTICS_TOPIC);
    // Time for publisher to connect to subscriber
    ros::Duration(0.5).sleep();
    PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)
    ros::Duration(0.1).sleep();
    ASSERT_TRUE(last_msg_.get());
    EXPECT_THAT(getVariables(*last_msg_),
                UnorderedElementsAre("macro_var1", "macro_var1_bk", "macro_var2"));
  }
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)
  ros::Duration(0.1).sleep();
  ASSERT_TRUE(last_msg_.get());
  EXPECT_THAT(getVariables(*last_msg_), UnorderedElementsAre("macro_var1", "macro_var2"));
}



}  // namespace pal

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pal_statistics_test");
  // first nodehandle created of an app must exist until the end of the life of the node
  // If not, you'll have funny stuff such as no logs printed
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
