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
namespace pal_statistics
{
class PalStatisticsTest : public ::testing::Test
{
public:
  PalStatisticsTest()
  {
    var1_ = 0.0;
    var2_ = 0.5;
    container_.resize(5);

    nh_.setCallbackQueue(&queue_);
    sub_ = nh_.subscribe(std::string(DEFAULT_STATISTICS_TOPIC) + "/full", 1, &PalStatisticsTest::fullTopicCb, this);
    names_sub_ = nh_.subscribe(std::string(DEFAULT_STATISTICS_TOPIC) + "/names", 1, &PalStatisticsTest::namesTopicCb, this);
    values_sub_ = nh_.subscribe(std::string(DEFAULT_STATISTICS_TOPIC) + "/values", 1, &PalStatisticsTest::valuesTopicCb, this);
  }
  
  void fullTopicCb(const pal_statistics_msgs::StatisticsConstPtr &msg)
  {
    last_msg_ = msg;
  }
  void namesTopicCb(const pal_statistics_msgs::StatisticsNamesConstPtr &msg)
  {
    last_names_msg_ = msg;
  }
  void valuesTopicCb(const pal_statistics_msgs::StatisticsValuesConstPtr &msg)
  {
    last_values_msg_ = msg;
  }
  void resetMsgs()
  {
    last_msg_.reset();
    // last names should not be reset, because it's not always published and you'll be using the last one
    last_values_msg_.reset();
  }
  bool waitForMsg(const ros::Duration &timeout)
  {
    ros::Time end = ros::Time::now() + timeout;
    while (ros::Time::now() < end)
    {
      queue_.callAvailable();
      ros::Duration(1e-4).sleep();
      if (last_msg_.get() && last_values_msg_.get()) 
      {
        return true;
      }
    }
    return false;
  }

protected:
  double var1_;
  double var2_;
  std::vector<int> container_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  ros::Subscriber names_sub_;
  ros::Subscriber values_sub_;
  pal_statistics_msgs::StatisticsConstPtr last_msg_;
  pal_statistics_msgs::StatisticsNamesConstPtr last_names_msg_;
  pal_statistics_msgs::StatisticsValuesConstPtr last_values_msg_;
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
  customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var1", &var1_);
  registry->unregisterVariable("var1");
  registry->unregisterVariable("var1");
}

TEST_F(PalStatisticsTest, checkValues)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);

  customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var2", &var2_);

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
  const unsigned short us = std::numeric_limits<unsigned short>::max();
  const char c = std::numeric_limits<char>::min();
  const unsigned char uc = std::numeric_limits<unsigned char>::max();
  int i = std::numeric_limits<int>::min();
  const unsigned int ui = std::numeric_limits<unsigned int>::max();
  long l = std::numeric_limits<long>::min();
  const unsigned long ul = std::numeric_limits<long>::max();
  long long ll = std::numeric_limits<long long>::min();
  const unsigned long long ull = std::numeric_limits<unsigned long long>::max();
  float min_f = std::numeric_limits<float>::min();
  const float max_f = std::numeric_limits<float>::max();
  const double min_d = std::numeric_limits<double>::min();
  double max_d = std::numeric_limits<double>::max();
  bool true_b = true;
  bool false_b = false;



  customRegister(*registry, "s", &s);
  customRegister(*registry, "us", &us);
  customRegister(*registry, "c", &c);
  customRegister(*registry, "uc", &uc);
  customRegister(*registry, "i", &i);
  customRegister(*registry, "ui", &ui);
  customRegister(*registry, "l", &l);
  customRegister(*registry, "ul", &ul);
  customRegister(*registry, "ll", &ll);
  customRegister(*registry, "ull", &ull);
  customRegister(*registry, "min_f", &min_f);
  customRegister(*registry, "max_f", &max_f);
  customRegister(*registry, "min_d", &min_d);
  customRegister(*registry, "max_d", &max_d);
  customRegister(*registry, "true_b", &true_b);
  customRegister(*registry, "false_b", &false_b);
  customRegister(*registry, "container_size", boost::function<size_t ()>(boost::bind(&std::vector<int>::size,
                                                           &container_)));
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

  IdType var1_id = customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var2", &var2_);
  customRegister(*registry, "container_size", boost::function<size_t ()>(boost::bind(&std::vector<int>::size,
                                                           &container_)));

  pal_statistics_msgs::Statistics msg = registry->createMsg();

  EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("var1", "var2", "container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));

  registry->unregisterVariable("var2");
  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("var1", "container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
  
  EXPECT_TRUE(registry->disable(var1_id));
  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
  
  
  EXPECT_TRUE(registry->enable(var1_id));
  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("var1", "container_size",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}


TEST_F(PalStatisticsTest, automaticRegistration)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  pal_statistics_msgs::Statistics msg;
  {
    RegistrationsRAII bookkeeping;

    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

    msg = registry->createMsg();

    EXPECT_NEAR(ros::Time::now().toSec(), msg.header.stamp.toSec(), 0.001);
    EXPECT_THAT(getVariables(msg),
                UnorderedElementsAre("var1", "var2",
                                     "topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.publish_buffer_full_errors",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
    
    EXPECT_TRUE(bookkeeping.disableAll());
    msg = registry->createMsg();
    EXPECT_THAT(getVariables(msg),
                UnorderedElementsAre("topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.publish_buffer_full_errors",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
    
    EXPECT_TRUE(bookkeeping.enableAll());
    msg = registry->createMsg();
    EXPECT_THAT(getVariables(msg),
                UnorderedElementsAre("var1", "var2", 
                                     "topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.publish_buffer_full_errors",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
    
  }

  msg = registry->createMsg();
  EXPECT_THAT(getVariables(msg),
              UnorderedElementsAre("topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}


TEST_F(PalStatisticsTest, automaticRegistrationDestruction)
{
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  {
    RegistrationsRAII bookkeeping;

    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

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
    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

    registry->publishAsync();
    ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));

    auto s = getVariableAndValues(*last_msg_);
    EXPECT_EQ(var1_, s["var1"]);
    EXPECT_EQ(var2_, s["var2"]);

    last_msg_.reset();
    ASSERT_FALSE(waitForMsg(ros::Duration(0.3)))
        << " Data shouldn't have been published because there were no calls to publishAsync";

    var1_ = 2.0;
    var2_ = 3.0;
    registry->publishAsync();
    ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));

    s = getVariableAndValues(*last_msg_);
    EXPECT_EQ(var1_, s["var1"]);
    EXPECT_EQ(var2_, s["var2"]);

    last_msg_.reset();
  }
  registry->publishAsync();
  ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
  // Number of internal statistics
  EXPECT_EQ(4, last_msg_->statistics.size());
}

TEST_F(PalStatisticsTest, macroTest)
{
  {
    RegistrationsRAII bookkeeping;
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1", &var1_, NULL);
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1_bk", &var1_, &bookkeeping);
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2", &var2_, NULL);
    REGISTER_VARIABLE_SIMPLE(DEFAULT_STATISTICS_TOPIC, &var2_, &bookkeeping);
    START_PUBLISH_THREAD(DEFAULT_STATISTICS_TOPIC);
    // Time for publisher to connect to subscriber
    ros::Duration(0.5).sleep();
    PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)
    ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
    EXPECT_THAT(getVariables(*last_msg_),
                UnorderedElementsAre("macro_var1", "macro_var1_bk", "macro_var2", "&var2_",
                                     "topic_stats.pal_statistics.publish_async_attempts",
                                     "topic_stats.pal_statistics.publish_async_failures",
                                     "topic_stats.pal_statistics.publish_buffer_full_errors",
                                     "topic_stats.pal_statistics.last_async_pub_duration"));
    last_msg_.reset();
  }
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC)
  ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
  EXPECT_THAT(getVariables(*last_msg_),
              UnorderedElementsAre("macro_var1", "macro_var2",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
  last_msg_.reset();
  
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2_bis", &var2_, NULL);
  UNREGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2", NULL);
  var1_ = 123.456;
  PUBLISH_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
  EXPECT_THAT(getVariables(*last_msg_),
              UnorderedElementsAre("macro_var1", "macro_var2_bis",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
  EXPECT_EQ(var1_, getVariableAndValues(*last_msg_)["macro_var1"]);
  UNREGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1", NULL);
  UNREGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2_bis", NULL);
  
  
  
  
}

void registerThread(boost::shared_ptr<StatisticsRegistry> registry, const std::string &prefix,
                    size_t iterations, double *variable, RegistrationsRAII *bookkeeping = NULL)
{
  for (size_t i = 0; i < iterations; ++i)
  {
    ros::Time b = ros::Time::now();
    customRegister(*registry, prefix + std::to_string(i), variable, bookkeeping);
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

TEST_F(PalStatisticsTest, stressAsync)
{
  double d = 5.0;
  boost::shared_ptr<StatisticsRegistry> registry =
      boost::make_shared<StatisticsRegistry>(DEFAULT_STATISTICS_TOPIC);
  registry->startPublishThread();
  customRegister(*registry, "test_variable", &d);
  customRegister(*registry, "test_variable2", &d);
  size_t received_messages = 0;
  
  ros::NodeHandle async_nh;
  ros::CallbackQueue async_queue;
  async_nh.setCallbackQueue(&async_queue);
  ros::AsyncSpinner spinner(1, &async_queue);  
  spinner.start();
  ros::Subscriber sub = async_nh.subscribe<pal_statistics_msgs::Statistics>(
      std::string(DEFAULT_STATISTICS_TOPIC) + "/full", 1000000,
      [&](const pal_statistics_msgs::StatisticsConstPtr &) { received_messages++; });
  while (sub.getNumPublishers() == 0)
  {
    ros::Duration(0.1).sleep();
  }
  
  ros::Rate rate(1e4);
  size_t num_messages = 3e4;
  size_t success_async = 0;
  for (size_t i = 0; i < num_messages; ++i)
  {
    success_async += registry->publishAsync();
    rate.sleep();
  }
  
  //Allow time for everything to arrive
  ros::Duration(0.5).sleep();
  
  EXPECT_EQ(success_async - registry->registration_list_.overwritten_data_count_, received_messages);  
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

  EXPECT_EQ(4 + n_variables * n_threads, msg.statistics.size());
  ROS_INFO_STREAM("Start publishAsync");
  ros::Time b = ros::Time::now();
  size_t iter = 10000;
//  ros::Rate rate(1e3);
  for (size_t i = 0; i < iter; ++i)
  {
    registry->publishAsync();
//    rate.sleep();
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
  EXPECT_EQ(4, msg.statistics.size());


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
  
  waitForMsg(ros::Duration(0.2));
  EXPECT_TRUE(last_msg_.get());
  ASSERT_EQ(1, last_msg_->statistics.size());
  EXPECT_EQ("single_stat", last_msg_->statistics[0].name);
  EXPECT_DOUBLE_EQ(d, last_msg_->statistics[0].value);
}



TEST_F(PalStatisticsTest, chaosTest)
{
  // Tests the registration of a variable and publication by the nonrt thread
  // before a publish_async has been performed
  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var1", &var1_, &bookkeeping);
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var2", &var2_, &bookkeeping);
  ros::Duration(0.2).sleep();
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  ros::Duration(0.2).sleep();  
}

TEST_F(PalStatisticsTest, chaosTest2)
{
  // Tests the unregistration of a variable and publication by the nonrt thread
  // before a publish_async has been performed
  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var1", &var1_, nullptr);
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var2", &var2_, &bookkeeping);
  UNREGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var1", nullptr);
  ros::Duration(0.2).sleep();
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  
  last_msg_.reset();
  ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
  EXPECT_THAT(getVariables(*last_msg_),
              UnorderedElementsAre("var2",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}

TEST_F(PalStatisticsTest, chaosTest3)
{
  // Tests the disabling of a variable and publication by the nonrt thread
  // before a publish_async has been performed
  RegistrationsRAII bookkeeping;
  auto var1id = REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var1", &var1_, nullptr);
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var2", &var2_, &bookkeeping);
  getRegistry(DEFAULT_STATISTICS_TOPIC)->disable(var1id);
  ros::Duration(0.2).sleep();
  PUBLISH_ASYNC_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  
  last_msg_.reset();
  ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
  EXPECT_THAT(getVariables(*last_msg_),
              UnorderedElementsAre("var2",
                                   "topic_stats.pal_statistics.publish_async_attempts",
                                   "topic_stats.pal_statistics.publish_async_failures",
                                   "topic_stats.pal_statistics.publish_buffer_full_errors",
                                   "topic_stats.pal_statistics.last_async_pub_duration"));
}


TEST_F(PalStatisticsTest, splitMsgTest)
{
  std::string topic("other_topic");
  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1", &var1_, NULL);
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var1_bk", &var1_, &bookkeeping);
  REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "macro_var2", &var2_, NULL);
  REGISTER_VARIABLE_SIMPLE(DEFAULT_STATISTICS_TOPIC, &var2_, &bookkeeping);
  // Time for publisher to connect to subscriber
  ros::Duration(0.2).sleep();
  unsigned int old_names_version = 0;
  for (int i = 0; i < 4; ++i)
  {
    for (bool remove : {false, true})
    {
      if (!remove)
      {
        REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var" + std::to_string(i), &var2_, NULL);
      }
      else
      {
        UNREGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "var" + std::to_string(i));
      }
      SCOPED_TRACE(std::string("Iteration ") + std::to_string(i) + " remove " +
                   std::to_string(remove));
      
      resetMsgs();
      PUBLISH_STATISTICS(DEFAULT_STATISTICS_TOPIC);
      ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
      ASSERT_EQ(last_msg_->header.stamp, last_names_msg_->header.stamp);
      ASSERT_EQ(last_names_msg_->header.stamp, last_values_msg_->header.stamp);
      ASSERT_EQ(last_names_msg_->names_version, last_values_msg_->names_version);
      ASSERT_GT(last_names_msg_->names_version, old_names_version);
      old_names_version = last_names_msg_->names_version;

      ASSERT_EQ(getVariables(*last_msg_), last_names_msg_->names);
      
      resetMsgs();
      PUBLISH_STATISTICS(DEFAULT_STATISTICS_TOPIC);
      ASSERT_TRUE(waitForMsg(ros::Duration(0.3)));
      ASSERT_LT(last_names_msg_->header.stamp, last_values_msg_->header.stamp);
      ASSERT_EQ(last_names_msg_->names_version, last_values_msg_->names_version);
      ASSERT_EQ(last_names_msg_->names_version, old_names_version);
      
    }
  }
}




}  // namespace pal_statistics

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pal_statistics_test");
  // first nodehandle created of an app must exist until the end of the life of the node
  // If not, you'll have funny stuff such as no logs printed
  ros::NodeHandle nh;
  ros::Time::waitForValid();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

