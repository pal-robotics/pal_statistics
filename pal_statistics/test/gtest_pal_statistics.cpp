/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/pal_statistics_macros.h>
#include <pal_statistics/pal_statistics.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <climits>
#include <cfloat>

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
    container_.resize(5);

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
  std::vector<int> container_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_;
  pal_statistics_msgs::StatisticsConstPtr last_msg_;
};


std::vector<std::string> getVariables(const pal_statistics_msgs::Statistics &msg)
{
  std::vector<std::string> v;
  for (const auto &s : msg.statistics)
  {
    v.push_back(s.name);
  }
  return v;
}

std::map<std::string, double> getVariableAndValues(const pal_statistics_msgs::Statistics &msg)
{
  std::map<std::string, double> m;
  for (const auto &s : msg.statistics)
  {
    m[s.name] = s.value;
  }
  return m;
}
TEST_F(PalStatisticsTest, misUse)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  registry->unregisterVariable("foo");
  registry->registerVariable("var1", &var1_);
  registry->registerVariable("var1", &var1_);
  registry->unregisterVariable("var1");
  registry->unregisterVariable("var1");
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

TEST_F(PalStatisticsTest, typeTest)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);


  short s = std::numeric_limits<short>::min();
  unsigned short us = std::numeric_limits<unsigned short>::max();
  char c = std::numeric_limits<char>::min();
  unsigned char uc = std::numeric_limits<unsigned char>::max();
  int i = std::numeric_limits<int>::min();
  unsigned int ui = std::numeric_limits<unsigned int>::max();
  long l = std::numeric_limits<long>::min();
  unsigned long ul = std::numeric_limits<long>::max();
  long long ll = std::numeric_limits<long long>::min();
  unsigned long long ull = std::numeric_limits<unsigned long long>::max();
  float min_f = std::numeric_limits<float>::min();
  float max_f = std::numeric_limits<float>::max();
  double min_d = std::numeric_limits<double>::min();
  double max_d = std::numeric_limits<double>::max();
  bool true_b = true;
  bool false_b = false;



  registry->registerVariable("s", &s);
  registry->registerVariable("us", &us);
  registry->registerVariable("c", &c);
  registry->registerVariable("uc", &uc);
  registry->registerVariable("i", &i);
  registry->registerVariable("ui", &ui);
  registry->registerVariable("l", &l);
  registry->registerVariable("ul", &ul);
  registry->registerVariable("ll", &ll);
  registry->registerVariable("ull", &ull);
  registry->registerVariable("min_f", &min_f);
  registry->registerVariable("max_f", &max_f);
  registry->registerVariable("min_d", &min_d);
  registry->registerVariable("max_d", &max_d);
  registry->registerVariable("true_b", &true_b);
  registry->registerVariable("false_b", &false_b);
  registry->registerFunction("container_size", boost::bind(&std::vector<int>::size,
                                                           &container_));
  pal_statistics_msgs::Statistics msg = registry->createMsg();

  auto values = getVariableAndValues(msg);
  EXPECT_EQ(s, values["s"]);
  EXPECT_EQ(us, values["us"]);
  EXPECT_EQ(c, values["c"]);
  EXPECT_EQ(uc, values["uc"]);
  EXPECT_EQ(i, values["i"]);
  EXPECT_EQ(ui, values["ui"]);
  EXPECT_EQ(l, values["l"]);
  EXPECT_EQ(ul, values["ul"]);
  EXPECT_EQ(ll, values["ll"]);
  EXPECT_EQ(ull, values["ull"]);
  EXPECT_EQ(min_f, values["min_f"]);
  EXPECT_EQ(max_f, values["max_f"]);
  EXPECT_EQ(min_d, values["min_d"]);
  EXPECT_EQ(max_d, values["max_d"]);
  EXPECT_EQ(true_b, values["true_b"]);
  EXPECT_EQ(false_b, values["false_b"]);
  EXPECT_EQ(container_.size(), values["container_size"]);

  EXPECT_NE(l, values["ul"]);
}

TEST_F(PalStatisticsTest, manualRegistration)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);

  registry->registerVariable("var1", &var1_);
  registry->registerVariable("var2", &var2_);
  registry->registerFunction("container_size", boost::bind(&std::vector<int>::size,
                                                           &container_));

  pal_statistics_msgs::Statistics msg = registry->createMsg();

  EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("var1", "var2", "container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));

  registry->unregisterVariable("var1");
  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("var2", "container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}


TEST_F(PalStatisticsTest, automaticRegistration)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  pal_statistics_msgs::Statistics msg;
  {
    RegistrationsRAII bookkeeping;

    registry->registerVariable("var1", &var1_, &bookkeeping);
    registry->registerVariable("var2", &var2_, &bookkeeping);

    msg = registry->createMsg();

    EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
    EXPECT_THAT(getVariables(msg),
                UnorderedElementsAre("var1", "var2", "topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
  }

  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}


TEST_F(PalStatisticsTest, automaticRegistrationDestruction)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  {
    RegistrationsRAII bookkeeping;

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
    RegistrationsRAII bookkeeping;

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
  // Number of internal statistics
  EXPECT_EQ(3, last_msg_->statistics.size());
}

TEST_F(PalStatisticsTest, macroTest)
{
  {
    RegistrationsRAII bookkeeping;
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
                UnorderedElementsAre("macro_var1", "macro_var1_bk", "macro_var2",
                                     "topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
  }
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)
  ros::Duration(0.1).sleep();
  ASSERT_TRUE(last_msg_.get());
  EXPECT_THAT(getVariables(*last_msg_),
              UnorderedElementsAre("macro_var1", "macro_var2",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}

void registerThread(boost::shared_ptr<StatisticsRegistry> registry, const std::string &prefix,
                    size_t iterations, double *variable, RegistrationsRAII *bookkeeping = NULL)
{
  for (size_t i = 0; i < iterations; ++i)
  {
    ros::Time b = ros::Time::now();
    registry->registerVariable(prefix + std::to_string(i), variable, bookkeeping);
    //    ROS_INFO_STREAM(i << " " << (ros::Time::now() - b).toSec());
  }
}
void unregisterThread(boost::shared_ptr<StatisticsRegistry> registry,
                      const std::string &prefix, size_t n_variables)
{
  // Deregister in inverse order
  for (size_t i = n_variables; i > 0; --i)
  {
    registry->unregisterVariable(prefix + std::to_string(i - 1), NULL);
  }
}


void publish(boost::shared_ptr<StatisticsRegistry> registry, size_t n_variables)
{
  for (size_t i = n_variables; i > 0; --i)
  {
    registry->publish();
  }
}


void publishAsync(boost::shared_ptr<StatisticsRegistry> registry, size_t n_variables)
{
  for (size_t i = n_variables; i > 0; --i)
  {
    registry->publishAsync();
  }
}


TEST_F(PalStatisticsTest, concurrencyTest)
{
  size_t n_threads = 5;
  size_t n_variables = 2e4/n_threads; //2e4 variables in total
  std::vector<boost::thread> threads;
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  registry->startPublishThread();
  ROS_INFO_STREAM("Start registration threads");
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads.push_back(boost::thread(boost::bind(&registerThread, registry,
                                                std::to_string(i) + "_", n_variables, &var1_,
                                                static_cast<RegistrationsRAII *>(NULL))));
  }
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads[i].join();
  }
  ROS_INFO_STREAM("Registration ended");
  pal_statistics_msgs::Statistics msg = registry->createMsg();

  EXPECT_EQ(3 + n_variables * n_threads, msg.statistics.size());
  ROS_INFO_STREAM("Start publishAsync");
  ros::Time b = ros::Time::now();
  size_t iter = 100000;
  for (size_t i = 0; i < iter; ++i)
  {
    registry->publishAsync();
  }
  // Time to publish 1000 times the registered statistics
  ROS_INFO_STREAM("End publishAsync " << (1000. * (ros::Time::now() - b).toSec()) / double(iter));
  ROS_INFO_STREAM("Start publish");
  for (size_t i = 0; i < n_variables; ++i)
  {
    registry->publish();
  }
  ROS_INFO_STREAM("End publish");

  threads.clear();
  ROS_INFO_STREAM("Start deregistration threads");
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads.push_back(boost::thread(
        boost::bind(&unregisterThread, registry, std::to_string(i) + "_", n_variables)));
  }
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads[i].join();
  }
  ROS_INFO_STREAM("Deregistration ended");
  msg = registry->createMsg();
  threads.clear();

  // Number of internal variables
  EXPECT_EQ(3, msg.statistics.size());


  for (size_t i = 0; i < n_threads; ++i)
  {
    threads[i].join();
  }

  threads.clear();
}

TEST_F(PalStatisticsTest, concurrencyMixTest)
{
  size_t n_variables = 2e3;
  size_t n_threads = 5;
  std::vector<boost::thread> threads;
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads.push_back(boost::thread(boost::bind(&registerThread, registry,
                                                std::to_string(i) + "_", n_variables, &var1_,
                                                static_cast<RegistrationsRAII *>(NULL))));
  }
  for (size_t i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }
  threads.clear();


  ROS_INFO_STREAM("Start thread mix");
  RegistrationsRAII bookkeeping;
  for (size_t i = 0; i < n_threads; ++i)
  {
    threads.push_back(boost::thread(
        boost::bind(&unregisterThread, registry, std::to_string(i) + "_", n_variables)));
    threads.push_back(boost::thread(boost::bind(&registerThread, registry,
                                                std::to_string(i + n_threads) + "_",
                                                n_variables, &var1_, &bookkeeping)));
    //    threads.push_back(boost::thread(boost::bind(&publish, registry, n_variables)));
    //    threads.push_back(boost::thread(boost::bind(&publishAsync, registry,
    //    n_variables)));
  }
  for (size_t i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }
  ROS_INFO_STREAM("End thread mix");
}

TEST_F(PalStatisticsTest, singlePublish)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  double d = 0.123;
  registry->publishCustomStatistic("single_stat", d);
  //Wait for message
  ros::Duration(0.5).sleep();


  EXPECT_TRUE(last_msg_.get());
  ASSERT_EQ(1, last_msg_->statistics.size());
  EXPECT_EQ("single_stat", last_msg_->statistics[0].name);
  EXPECT_DOUBLE_EQ(d, last_msg_->statistics[0].value);
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
