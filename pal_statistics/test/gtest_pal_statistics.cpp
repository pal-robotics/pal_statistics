// Copyright 2023 PAL Robotics S.L.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pal_statistics/pal_statistics_macros.hpp>
#include <pal_statistics/pal_statistics.hpp>
#include <registration_list.hpp>  // from src directory

using ::testing::Value;
using ::testing::UnorderedElementsAre;
using ::testing::DoubleEq;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::Pair;
using ::testing::Eq;
using std::placeholders::_1;
using pal_statistics::StatisticsRegistry;
using pal_statistics::RegistrationsRAII;
using pal_statistics::IdType;
using pal_statistics::getRegistry;

static const auto kNamesQoS = rclcpp::SystemDefaultsQoS()
  .keep_last(1u)
  .reliable()
  .transient_local();

static const auto kDataQoS = rclcpp::SystemDefaultsQoS()
  .keep_last(1u)
  .best_effort()
  .transient_local();

namespace
{
std::vector<std::string> getVariables(const pal_statistics_msgs::msg::Statistics & msg)
{
  std::vector<std::string> v;
  for (const auto & s : msg.statistics) {
    v.push_back(s.name);
  }
  return v;
}

std::map<std::string, double> getVariableAndValues(const pal_statistics_msgs::msg::Statistics & msg)
{
  std::map<std::string, double> m;
  for (const auto & s : msg.statistics) {
    m[s.name] = s.value;
  }
  return m;
}

template<typename M, typename W>
class WaitPredicateFormatterFromMatcher
{
public:
  explicit WaitPredicateFormatterFromMatcher(
    M m,
    W wait_func,
    std::chrono::milliseconds timeout)

  : matcher_(std::move(m))
    , wait_func_(std::move(wait_func))
    , timeout_(timeout)
  {}

  // This template () operator allows a WaitPredicateFormatterFromMatcher
  // object to act as a predicate-formatter suitable for using with
  // Google Test's EXPECT_PRED_FORMAT1() macro.
  template<typename F>
  ::testing::AssertionResult operator()(const char * value_text, const F & func_to_test) const
  {
    using ::testing::AssertionResult;
    using ::testing::SafeMatcherCast;
    using ::testing::Matcher;
    using ::testing::AssertionSuccess;
    using ::testing::AssertionFailure;
    using ::testing::StringMatchResultListener;
    using T = std::invoke_result_t<F>;
    // inspired by googletest's PredicateFormatterFromMatcher
    const Matcher<const T &> matcher = SafeMatcherCast<const T &>(matcher_);
    const auto end = std::chrono::steady_clock::now() + timeout_;

    while (rclcpp::ok()) {
      T && x = func_to_test();

      if (matcher.Matches(x)) {
        return AssertionSuccess();
      }

      wait_func_();

      const auto now = std::chrono::steady_clock::now();
      if (now > end) {
        ::std::stringstream ss;
        ss << "After " << timeout_.count() << " ms\n"
           << "Value of: " << value_text << "\n"
           << "Expected: ";
        matcher.DescribeTo(&ss);

        // Rerun the matcher to "PrintAndExain" the failure.
        StringMatchResultListener listener;
        if (MatchPrintAndExplain(x, matcher, &listener)) {
          ss << "\n  The matcher failed on the initial attempt; but passed when "
            "rerun to generate the explanation.";
        }
        ss << "\n  Actual: " << listener.str();
        return AssertionFailure() << ss.str();
      }
    }
    return AssertionSuccess();
  }

private:
  const M matcher_;
  W wait_func_;
  const std::chrono::milliseconds timeout_;
  WaitPredicateFormatterFromMatcher & operator=(WaitPredicateFormatterFromMatcher const &) = delete;
};

template<typename M>
inline auto
MakeWaitPredicateFormatterFromMatcher(
  M matcher, rclcpp::Executor::SharedPtr executor,
  std::chrono::milliseconds timeout = std::chrono::milliseconds{300})
{
  const auto wait_func = [executor] {
      executor->spin_some();
      rclcpp::sleep_for(std::chrono::nanoseconds{100});
    };

  return WaitPredicateFormatterFromMatcher(std::move(matcher), std::move(wait_func), timeout);
}

}  // namespace

#define ASSERT_EVENTUALLY_THAT(func_to_test, matcher, executor, timeout) ASSERT_PRED_FORMAT1( \
    MakeWaitPredicateFormatterFromMatcher(matcher, executor, timeout), func_to_test)

#define EXPECT_EVENTUALLY_THAT(func_to_test, matcher, executor, timeout) EXPECT_PRED_FORMAT1( \
    MakeWaitPredicateFormatterFromMatcher(matcher, executor, timeout), func_to_test)

template<typename NodeT>
class PalStatisticsTestHelperClass
{
public:
  explicit PalStatisticsTestHelperClass(const std::string & name)
  {
    node_ = std::make_shared<NodeT>(name);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    var1_ = 0.0;
    var2_ = 0.5;
    container_.resize(5);

    sub_ = node_->template create_subscription<pal_statistics_msgs::msg::Statistics>(
      std::string("~/") + DEFAULT_STATISTICS_TOPIC + "/full", kDataQoS,
      std::bind(&PalStatisticsTestHelperClass::fullTopicCb, this, std::placeholders::_1));
    names_sub_ = node_->template create_subscription<pal_statistics_msgs::msg::StatisticsNames>(
      std::string("~/") + DEFAULT_STATISTICS_TOPIC + "/names", kNamesQoS,
      std::bind(&PalStatisticsTestHelperClass::namesTopicCb, this, std::placeholders::_1));
    values_sub_ =
      node_->template create_subscription<pal_statistics_msgs::msg::StatisticsValues>(
      std::string("~/") + DEFAULT_STATISTICS_TOPIC + "/values", kDataQoS,
      std::bind(&PalStatisticsTestHelperClass::valuesTopicCb, this, std::placeholders::_1));
  }

  void fullTopicCb(const pal_statistics_msgs::msg::Statistics::SharedPtr msg)
  {
    last_msg_ = msg;
  }
  void namesTopicCb(const pal_statistics_msgs::msg::StatisticsNames::SharedPtr msg)
  {
    last_names_msg_ = msg;
  }
  void valuesTopicCb(const pal_statistics_msgs::msg::StatisticsValues::SharedPtr msg)
  {
    last_values_msg_ = msg;
  }
  void resetMsgs()
  {
    last_msg_.reset();
    // last names should not be reset, because it's not always published and
    // you'll be using the last one
    last_values_msg_.reset();
  }

  bool waitForMsg(const std::chrono::milliseconds & timeout = std::chrono::milliseconds{300})
  {
    rclcpp::Time end = node_->get_clock()->now() + timeout;
    while (rclcpp::ok() && node_->get_clock()->now() < end) {
      executor_->spin_some();
      rclcpp::sleep_for(std::chrono::nanoseconds(100));
      if (last_msg_.get() && last_values_msg_.get() && last_names_msg_.get()) {
        return true;
      }
    }
    return false;
  }

  // tests
  void misUseTest();
  void checkValuesTest();
  void typeTest();
  void manualRegistrationTest();
  void automaticRegistrationTest();
  void automaticRegistrationDestructionTest();
  void asyncPublisherTest();
  void macroTest();
  void stressAsyncTest();
  void concurrencyTest();
  void concurrencyMixTest();
  void singlePublishTest();
  void chaosTest();
  void chaosTest2();
  void chaosTest3();
  void chaosTest4();
  void scopedBookkeepingTest();
  void bookkeepingDisabledByDefault();
  void splitMsgTest();
  void callStartPublishThreadMultipleTimes();
  void startStopPublishThreadTest();

protected:
  double var1_;
  double var2_;
  std::shared_ptr<NodeT> node_;
  std::vector<int> container_;
  rclcpp::Executor::SharedPtr executor_;

  rclcpp::Subscription<pal_statistics_msgs::msg::Statistics>::SharedPtr sub_;
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr names_sub_;
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr values_sub_;

  pal_statistics_msgs::msg::Statistics::ConstSharedPtr last_msg_;
  pal_statistics_msgs::msg::StatisticsNames::ConstSharedPtr last_names_msg_;
  pal_statistics_msgs::msg::StatisticsValues::ConstSharedPtr last_values_msg_;
};

using PalStatisticsNodeTests = PalStatisticsTestHelperClass<rclcpp::Node>;
using PalStatisticsLifecycleNodeTests =
  PalStatisticsTestHelperClass<rclcpp_lifecycle::LifecycleNode>;

class PalStatisticsTest : public ::testing::Test
{
public:
  PalStatisticsTest() = default;
  virtual ~PalStatisticsTest() = default;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_test_ = std::make_unique<PalStatisticsNodeTests>("pal_statistics_node_test");
    lifecycle_test_ = std::make_unique<PalStatisticsLifecycleNodeTests>(
      "pal_statistics_lifecycle_test");
  }

  void TearDown() override
  {
    node_test_.reset();
    lifecycle_test_.reset();
  }

protected:
  std::unique_ptr<PalStatisticsNodeTests> node_test_;
  std::unique_ptr<PalStatisticsLifecycleNodeTests> lifecycle_test_;
};

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::misUseTest()
{
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(
    node_, std::string(
      node_->get_name()) + "/" + DEFAULT_STATISTICS_TOPIC);
  registry->unregisterVariable("foo");
  customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var1", &var1_);
  registry->unregisterVariable("var1");
  registry->unregisterVariable("var1");
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::checkValuesTest()
{
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(
    node_, std::string(
      node_->get_name()) + "/" + DEFAULT_STATISTICS_TOPIC);

  customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var2", &var2_);

  var1_ = 1.0;
  var2_ = 2.0;
  pal_statistics_msgs::msg::Statistics msg = registry->createMsg();

  EXPECT_NEAR(
    node_->get_clock()->now().seconds(),
    rclcpp::Time(msg.header.stamp).seconds(), 0.001);
  auto s = getVariableAndValues(msg);
  EXPECT_THAT(s, Contains(Pair("var1", DoubleEq(var1_))));
  EXPECT_THAT(s, Contains(Pair("var2", DoubleEq(var2_))));

  var1_ = 100.0;
  var2_ = -100.0;
  msg = registry->createMsg();
  s = getVariableAndValues(msg);
  EXPECT_THAT(s, Contains(Pair("var1", DoubleEq(var1_))));
  EXPECT_THAT(s, Contains(Pair("var2", DoubleEq(var2_))));
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::typeTest()
{
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(
    node_, std::string(
      node_->get_name()) + "/" + DEFAULT_STATISTICS_TOPIC);


  int16_t s = std::numeric_limits<int16_t>::min();
  const uint16_t us = std::numeric_limits<uint16_t>::max();
  const char c = std::numeric_limits<char>::min();
  const unsigned char uc = std::numeric_limits<unsigned char>::max();
  int i = std::numeric_limits<int>::min();
  const uint32_t ui = std::numeric_limits<uint32_t>::max();
  int64_t l = std::numeric_limits<int64_t>::min();
  const uint64_t ul = std::numeric_limits<int64_t>::max();
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
  customRegister(*registry, "min_f", &min_f);
  customRegister(*registry, "max_f", &max_f);
  customRegister(*registry, "min_d", &min_d);
  customRegister(*registry, "max_d", &max_d);
  customRegister(*registry, "true_b", &true_b);
  customRegister(*registry, "false_b", &false_b);
  customRegister(
    *registry, "container_size", std::function<size_t()>(
      std::bind(
        &std::vector<int>::size,
        &container_)));
  pal_statistics_msgs::msg::Statistics msg = registry->createMsg();

  auto values = getVariableAndValues(msg);
  EXPECT_EQ(s, static_cast<int16_t>(values["s"]));
  EXPECT_EQ(us, static_cast<uint16_t>(values["us"]));
  EXPECT_EQ(c, static_cast<char>(values["c"]));
  EXPECT_EQ(uc, static_cast<unsigned char>(values["uc"]));
  EXPECT_EQ(i, static_cast<int>(values["i"]));
  EXPECT_EQ(ui, static_cast<uint32_t>(values["ui"]));
  // Can't compare in original type, too big of a precision loss
  EXPECT_DOUBLE_EQ(l, values["l"]);
  EXPECT_DOUBLE_EQ(ul, values["ul"]);
  EXPECT_FLOAT_EQ(min_f, static_cast<float>(values["min_f"]));
  EXPECT_FLOAT_EQ(max_f, static_cast<float>(values["max_f"]));
  EXPECT_DOUBLE_EQ(min_d, values["min_d"]);
  EXPECT_DOUBLE_EQ(max_d, values["max_d"]);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
  EXPECT_EQ(true_b, static_cast<bool>(values["true_b"]));
  EXPECT_EQ(false_b, static_cast<bool>(values["false_b"]));
#pragma GCC diagnostic pop
  EXPECT_EQ(container_.size(), static_cast<uint64_t>(values["container_size"]));
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::manualRegistrationTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);

  IdType var1_id = customRegister(*registry, "var1", &var1_);
  customRegister(*registry, "var2", &var2_);
  customRegister(
    *registry, "container_size", std::function<size_t()>(
      std::bind(
        &std::vector<int>::size,
        &container_)));

  pal_statistics_msgs::msg::Statistics msg = registry->createMsg();

  EXPECT_NEAR(node_->get_clock()->now().seconds(), rclcpp::Time(msg.header.stamp).seconds(), 0.001);
  EXPECT_THAT(
    getVariables(msg),
    UnorderedElementsAre(
      "var1", "var2", "container_size",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"));

  registry->unregisterVariable("var2");
  msg = registry->createMsg();
  EXPECT_THAT(
    getVariables(msg),
    UnorderedElementsAre(
      "var1", "container_size",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"));

  EXPECT_TRUE(registry->disable(var1_id));
  msg = registry->createMsg();
  EXPECT_THAT(
    getVariables(msg),
    UnorderedElementsAre(
      "container_size",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"));


  EXPECT_TRUE(registry->enable(var1_id));
  msg = registry->createMsg();
  EXPECT_THAT(
    getVariables(msg),
    UnorderedElementsAre(
      "var1", "container_size",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"));
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::automaticRegistrationTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);
  pal_statistics_msgs::msg::Statistics msg;
  {
    RegistrationsRAII bookkeeping;

    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

    msg = registry->createMsg();

    EXPECT_NEAR(
      node_->get_clock()->now().seconds(), rclcpp::Time(msg.header.stamp).seconds(),
      0.001);
    EXPECT_THAT(
      getVariables(msg),
      UnorderedElementsAre(
        "var1", "var2",
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"));

    EXPECT_TRUE(bookkeeping.disableAll());
    msg = registry->createMsg();
    EXPECT_THAT(
      getVariables(msg),
      UnorderedElementsAre(
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"));

    EXPECT_TRUE(bookkeeping.enableAll());
    msg = registry->createMsg();
    EXPECT_THAT(
      getVariables(msg),
      UnorderedElementsAre(
        "var1", "var2",
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"));
  }

  msg = registry->createMsg();
  EXPECT_THAT(
    getVariables(msg),
    UnorderedElementsAre(
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"));
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::automaticRegistrationDestructionTest()
{
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(
    node_, std::string(
      node_->get_name()) + "/" + DEFAULT_STATISTICS_TOPIC);
  {
    RegistrationsRAII bookkeeping;

    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

    registry->createMsg();

    // Delete the main class, the bookkeeper shouldn't crash on destruction
    registry.reset();
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::asyncPublisherTest()
{
  constexpr auto timeout = std::chrono::milliseconds{300};
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(
    node_, std::string(
      node_->get_name()) + "/" + DEFAULT_STATISTICS_TOPIC);

  const auto promised_publication = [&]() {
      return registry->publishAsync();
    };

  {
    RegistrationsRAII bookkeeping;

    registry->startPublishThread();

    // Time for publisher to connect to subscriber
    while (sub_->get_publisher_count() == 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    customRegister(*registry, "var1", &var1_, &bookkeeping);
    customRegister(*registry, "var2", &var2_, &bookkeeping);

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    const auto publication_of_var1_var2_stats = [&]() -> std::map<std::string, double> {
        const auto is_msg_received = last_msg_.get() &&
          last_values_msg_.get() &&
          last_names_msg_.get();

        if (!is_msg_received) {
          return {};
        }
        return getVariableAndValues(*last_msg_);
      };

    EXPECT_EVENTUALLY_THAT(
      publication_of_var1_var2_stats,
      AllOf(
        Contains(
          Pair("var1", DoubleEq(var1_))),
        Contains(
          Pair("var2", DoubleEq(var2_)))),
      executor_, timeout);

    last_msg_.reset();

    ASSERT_FALSE(waitForMsg()) <<
      " Data shouldn't have been published because there were no calls to publishAsync";

    var1_ = 2.0;
    var2_ = 3.0;

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    ASSERT_TRUE(waitForMsg());

    EXPECT_EVENTUALLY_THAT(
      publication_of_var1_var2_stats,
      AllOf(
        Contains(
          Pair("var1", DoubleEq(var1_))),
        Contains(
          Pair("var2", DoubleEq(var2_)))),
      executor_, timeout);

    last_msg_.reset();
  }

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  ASSERT_TRUE(waitForMsg());
  // Number of internal statistics
  EXPECT_EQ(4u, last_msg_->statistics.size());
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::macroTest()
{
  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic)
    };

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  {
    RegistrationsRAII bookkeeping;
    REGISTER_VARIABLE(node_, statistics_topic, "macro_var1", &var1_, NULL);
    REGISTER_VARIABLE(node_, statistics_topic, "macro_var1_bk", &var1_, &bookkeeping);
    REGISTER_VARIABLE(node_, statistics_topic, "macro_var2", &var2_, NULL);
    REGISTER_VARIABLE_SIMPLE(node_, statistics_topic, &var2_, &bookkeeping);
    START_PUBLISH_THREAD(node_, statistics_topic);
    // Time for publisher to connect to subscriber
    while (sub_->get_publisher_count() == 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    EXPECT_EVENTUALLY_THAT(
      get_variables,
      UnorderedElementsAre(
        "macro_var1",
        "macro_var1_bk",
        "macro_var2",
        "&var2_",
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"),
      executor_, timeout);
    last_msg_.reset();
  }

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  EXPECT_EVENTUALLY_THAT(
    get_variables,
    UnorderedElementsAre(
      "macro_var1", "macro_var2",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"),
    executor_, timeout);

  last_msg_.reset();

  REGISTER_VARIABLE(node_, statistics_topic, "macro_var2_bis", &var2_, NULL);
  UNREGISTER_VARIABLE(node_, statistics_topic, "macro_var2", NULL);
  var1_ = 123.456;

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  EXPECT_EVENTUALLY_THAT(
    get_variables,
    UnorderedElementsAre(
      "macro_var1", "macro_var2_bis",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"),
    executor_, timeout);

  EXPECT_THAT(getVariableAndValues(*last_msg_), Contains(Pair("macro_var1", DoubleEq(var1_))));
  UNREGISTER_VARIABLE(node_, statistics_topic, "macro_var1", NULL);
  UNREGISTER_VARIABLE(node_, statistics_topic, "macro_var2_bis", NULL);
}

TEST_F(PalStatisticsTest, misUse)
{
  node_test_->misUseTest();
  lifecycle_test_->misUseTest();
}

TEST_F(PalStatisticsTest, checkValues)
{
  node_test_->checkValuesTest();
  lifecycle_test_->checkValuesTest();
}

TEST_F(PalStatisticsTest, typeTest)
{
  node_test_->typeTest();
  lifecycle_test_->typeTest();
}

TEST_F(PalStatisticsTest, manualRegistration)
{
  node_test_->manualRegistrationTest();
  lifecycle_test_->manualRegistrationTest();
}

TEST_F(PalStatisticsTest, automaticRegistration)
{
  node_test_->automaticRegistrationTest();
  lifecycle_test_->automaticRegistrationTest();
}

TEST_F(PalStatisticsTest, automaticRegistrationDestruction)
{
  node_test_->automaticRegistrationDestructionTest();
  lifecycle_test_->automaticRegistrationDestructionTest();
}

TEST_F(PalStatisticsTest, asyncPublisher)
{
  node_test_->asyncPublisherTest();
  lifecycle_test_->asyncPublisherTest();
}

TEST_F(PalStatisticsTest, macroTest)
{
  node_test_->macroTest();
  lifecycle_test_->macroTest();
}

void registerThread(
  std::shared_ptr<StatisticsRegistry> registry, const std::string & prefix,
  size_t iterations, double * variable, RegistrationsRAII * bookkeeping = NULL)
{
  for (size_t i = 0; i < iterations; ++i) {
    customRegister(*registry, prefix + std::to_string(i), variable, bookkeeping);
    //    RCLCPP_INFO_STREAM(node_->get_logger(), i << " " << (ros::Time::now() - b).toSec());
  }
}
void unregisterThread(
  std::shared_ptr<StatisticsRegistry> registry,
  const std::string & prefix, size_t n_variables)
{
  // Deregister in inverse order
  for (size_t i = n_variables; i > 0; --i) {
    registry->unregisterVariable(prefix + std::to_string(i - 1), NULL);
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::stressAsyncTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  double d = 5.0;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);
  registry->startPublishThread();
  customRegister(*registry, "test_variable", &d);
  customRegister(*registry, "test_variable2", &d);
  size_t received_messages = 0;

  // Create a callback group to spin independently from the rest of callbacks of the node
  auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto group_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  group_executor->add_callback_group(cb_group, node_->get_node_base_interface());


  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group;
  auto topic_sub = node_->template create_subscription<pal_statistics_msgs::msg::Statistics>(
    std::string(statistics_topic) + "/full", kDataQoS,
    [&](const pal_statistics_msgs::msg::Statistics::SharedPtr) {received_messages++;}, sub_opts);

  auto future_handle = std::async(
    std::launch::async, [&group_executor]() -> void {
      group_executor->spin();
    });


  while (topic_sub->get_publisher_count() == 0) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::Rate rate(std::chrono::milliseconds(10));
  size_t num_messages = 1000;
  size_t success_async = 0;
  for (size_t i = 0; i < num_messages; ++i) {
    success_async += registry->publishAsync();
    rate.sleep();
  }

  // Allow time for everything to arrive
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  group_executor->cancel();

  // Reliability is not guaranteed, allow for some error
  EXPECT_NEAR(
    success_async - registry->registration_list_->overwritten_data_count_,
    received_messages, 50);
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::concurrencyTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  size_t n_threads = 5;
  size_t n_variables = 2e4 / n_threads;  // 2e4 variables in total
  std::vector<std::thread> threads;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);
  registry->startPublishThread();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Start registration threads");
  for (size_t i = 0; i < n_threads; ++i) {
    threads.push_back(
      std::thread(
        std::bind(
          &registerThread, registry,
          std::to_string(i) + "_", n_variables, &var1_,
          static_cast<RegistrationsRAII *>(NULL))));
  }
  for (size_t i = 0; i < n_threads; ++i) {
    threads[i].join();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Registration ended");
  pal_statistics_msgs::msg::Statistics msg = registry->createMsg();

  EXPECT_EQ(4 + n_variables * n_threads, msg.statistics.size());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Start publishAsync");
  rclcpp::Time b = node_->get_clock()->now();
  size_t iter = 10000;
  // ros::Rate rate(1e3);
  for (size_t i = 0; i < iter; ++i) {
    registry->publishAsync();
    // rate.sleep();
  }
  // Time to publish 1000 times the registered statistics
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "End publishAsync " << (1000. * (node_->get_clock()->now() - b).seconds()) /
      static_cast<double>(iter));
  RCLCPP_INFO_STREAM(node_->get_logger(), "Start publish");
  for (size_t i = 0; i < n_variables; ++i) {
    registry->publish();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "End publish");

  threads.clear();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Start deregistration threads");
  for (size_t i = 0; i < n_threads; ++i) {
    threads.push_back(
      std::thread(
        std::bind(&unregisterThread, registry, std::to_string(i) + "_", n_variables)));
  }
  for (size_t i = 0; i < n_threads; ++i) {
    threads[i].join();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Deregistration ended");
  msg = registry->createMsg();
  threads.clear();

  // Number of internal variables
  EXPECT_EQ(4u, msg.statistics.size());
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::concurrencyMixTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  size_t n_variables = 2e3;
  size_t n_threads = 5;
  std::vector<std::thread> threads;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);
  for (size_t i = 0; i < n_threads; ++i) {
    threads.push_back(
      std::thread(
        std::bind(
          &registerThread, registry,
          std::to_string(i) + "_", n_variables, &var1_,
          static_cast<RegistrationsRAII *>(NULL))));
  }
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].join();
  }
  threads.clear();


  RCLCPP_INFO_STREAM(node_->get_logger(), "Start thread mix");
  RegistrationsRAII bookkeeping;
  for (size_t i = 0; i < n_threads; ++i) {
    threads.push_back(
      std::thread(
        std::bind(&unregisterThread, registry, std::to_string(i) + "_", n_variables)));
    threads.push_back(
      std::thread(
        std::bind(
          &registerThread, registry,
          std::to_string(i + n_threads) + "_",
          n_variables, &var1_, &bookkeeping)));
    //    threads.push_back(std::thread(std::bind(&publish, registry, n_variables)));
    //    threads.push_back(std::thread(std::bind(&publishAsync, registry,
    //    n_variables)));
  }
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].join();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "End thread mix");
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::singlePublishTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  std::shared_ptr<StatisticsRegistry> registry =
    std::make_shared<StatisticsRegistry>(node_, statistics_topic);
  double d = 0.123;
  registry->publishCustomStatistic("single_stat", d);

  waitForMsg();
  EXPECT_TRUE(last_msg_.get());
  ASSERT_EQ(1u, last_msg_->statistics.size());
  EXPECT_EQ("single_stat", last_msg_->statistics[0].name);
  EXPECT_DOUBLE_EQ(d, last_msg_->statistics[0].value);
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::chaosTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;

  // Tests the registration of a variable and publication by the nonrt thread
  // before a publish_async has been performed
  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(node_, statistics_topic, "var1", &var1_, &bookkeeping);
  PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
  REGISTER_VARIABLE(node_, statistics_topic, "var2", &var2_, &bookkeeping);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::chaosTest2()
{
  // Tests the unregistration of a variable and publication by the nonrt thread
  // before a publish_async has been performed

  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
    };

  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(node_, statistics_topic, "var1", &var1_, nullptr);

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  REGISTER_VARIABLE(node_, statistics_topic, "var2", &var2_, &bookkeeping);
  UNREGISTER_VARIABLE(node_, statistics_topic, "var1", nullptr);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  last_msg_.reset();

  /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
  /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
  /// To prevent flakiness, we wait for a reasonable amount of time
  /// that the promised statistics gets actually published
  EXPECT_EVENTUALLY_THAT(
    get_variables,
    UnorderedElementsAre(
      "var2",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"),
    executor_, timeout)
    << "'var2' has not been published yet after " << timeout.count() << " ms";
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::chaosTest3()
{
  // Tests the disabling of a variable and publication by the nonrt thread
  // before a publish_async has been performed
  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
    };

  RegistrationsRAII bookkeeping;
  auto var1id = REGISTER_VARIABLE(node_, statistics_topic, "var1", &var1_, nullptr);

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  REGISTER_VARIABLE(node_, statistics_topic, "var2", &var2_, &bookkeeping);
  getRegistry(node_, statistics_topic)->disable(var1id);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  last_msg_.reset();

  /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
  /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
  /// To prevent flakiness, we wait for a reasonable amount of time
  /// that the promised statistics gets actually published
  EXPECT_EVENTUALLY_THAT(
    get_variables,
    UnorderedElementsAre(
      "var2",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"),
    executor_, timeout)
    << "'var2' has not been published yet after " << timeout.count() << " ms";
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::chaosTest4()
{
  // Tests the unregistration of a variable and publication by the nonrt thread
  // before a publish_async has been performed

  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
    };

  const std::string registry_key = node_->get_node_topics_interface()->resolve_topic_name(
    statistics_topic);
  INITIALIZE_REGISTRY(node_, statistics_topic, registry_key);

  ASSERT_TRUE(pal_statistics::getRegistry(registry_key));

  RegistrationsRAII bookkeeping;
  REGISTER_ENTITY(registry_key, "var123", &var1_);

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  REGISTER_ENTITY(registry_key, "var2", &var2_, &bookkeeping);
  REGISTER_ENTITY(registry_key, "var3", &var2_, &bookkeeping);
  UNREGISTER_ENTITY(registry_key, "var123");
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
    << "Unable to publish stats variables after " << timeout.count() << " ms";

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  last_msg_.reset();

  /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
  /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
  /// To prevent flakiness, we wait for a reasonable amount of time
  /// that the promised statistics gets actually published
  EXPECT_EVENTUALLY_THAT(
    get_variables,
    UnorderedElementsAre(
      "var2", "var3",
      "topic_stats." + statistics_topic + ".publish_async_attempts",
      "topic_stats." + statistics_topic + ".publish_async_failures",
      "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
      "topic_stats." + statistics_topic + ".last_async_pub_duration"),
    executor_, timeout)
    << "'var2' has not been published yet after " << timeout.count() << " ms";
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::scopedBookkeepingTest()
{
  // Tests the unregistration of a variable and publication by the nonrt thread
  // before a publish_async has been performed

  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
    };

  const std::string registry_key = node_->get_node_topics_interface()->resolve_topic_name(
    statistics_topic);
  INITIALIZE_REGISTRY(node_, statistics_topic, registry_key);

  ASSERT_TRUE(pal_statistics::getRegistry(registry_key));

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  {
    RegistrationsRAII bookkeeping_1;
    {
      RegistrationsRAII bookkeeping_2;
      REGISTER_ENTITY(registry_key, "var123", &var1_, &bookkeeping_2);
      REGISTER_ENTITY(registry_key, "var1234", &var1_, &bookkeeping_1);

      ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
        << "Unable to publish stats variables after " << timeout.count() << " ms";

      REGISTER_ENTITY(registry_key, "var2", &var2_, &bookkeeping_1);
      REGISTER_ENTITY(registry_key, "var3", &var1_, &bookkeeping_1);
      REGISTER_ENTITY(registry_key, "var4", &var2_, &bookkeeping_2);
      REGISTER_ENTITY(registry_key, "var5", &var1_, &bookkeeping_2);
      UNREGISTER_ENTITY(registry_key, "var123");
      UNREGISTER_ENTITY(registry_key, "var1234");

      rclcpp::sleep_for(std::chrono::milliseconds(200));

      ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
        << "Unable to publish stats variables after " << timeout.count() << " ms";

      last_msg_.reset();

      /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
      /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
      /// To prevent flakiness, we wait for a reasonable amount of time
      /// that the promised statistics gets actually published
      EXPECT_EVENTUALLY_THAT(
        get_variables,
        UnorderedElementsAre(
          "var2", "var3", "var4", "var5",
          "topic_stats." + statistics_topic + ".publish_async_attempts",
          "topic_stats." + statistics_topic + ".publish_async_failures",
          "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
          "topic_stats." + statistics_topic + ".last_async_pub_duration"),
        executor_, timeout)
        << "'var2' has not been published yet after " << timeout.count() << " ms";
    }

    // As the bookkeeping_2 is out of scope, the variables registered with it should be unregistered

    rclcpp::sleep_for(std::chrono::milliseconds(200));

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    last_msg_.reset();

    /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
    /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
    /// To prevent flakiness, we wait for a reasonable amount of time
    /// that the promised statistics gets actually published
    EXPECT_EVENTUALLY_THAT(
      get_variables,
      UnorderedElementsAre(
        "var2", "var3",
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"),
      executor_, timeout)
      << "'var2' has not been published yet after " << timeout.count() << " ms";
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::bookkeepingDisabledByDefault()
{
  // Tests the unregistration of a variable and publication by the nonrt thread
  // before a publish_async has been performed

  constexpr auto timeout = std::chrono::milliseconds{300};
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  const auto promised_publication = [&]() {
      return PUBLISH_ASYNC_STATISTICS(node_, statistics_topic);
    };

  const std::string registry_key = node_->get_node_topics_interface()->resolve_topic_name(
    statistics_topic);
  INITIALIZE_REGISTRY(node_, statistics_topic, registry_key);

  ASSERT_TRUE(pal_statistics::getRegistry(registry_key));

  const auto get_variables = [this]() -> std::vector<std::string> {
      if (!last_msg_) {
        return {};
      }
      return getVariables(*last_msg_);
    };

  {
    RegistrationsRAII bookkeeping;
    REGISTER_ENTITY(registry_key, "var2", &var2_, &bookkeeping, false);
    REGISTER_ENTITY(registry_key, "var3", &var1_, &bookkeeping, false);
    REGISTER_ENTITY(registry_key, "var1234", &var1_, &bookkeeping, false);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    last_msg_.reset();

    /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
    /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
    /// To prevent flakiness, we wait for a reasonable amount of time
    /// that the promised statistics gets actually published
    EXPECT_EVENTUALLY_THAT(
      get_variables,
      UnorderedElementsAre(
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"),
      executor_, timeout)
      << "Nothing should be published other than standard as they are disabled when registering " <<
      timeout.count() << " ms";

    bookkeeping.enableAll();
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    last_msg_.reset();

    /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
    /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
    /// To prevent flakiness, we wait for a reasonable amount of time
    /// that the promised statistics gets actually published
    EXPECT_EVENTUALLY_THAT(
      get_variables,
      UnorderedElementsAre(
        "var2", "var3", "var1234",
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"),
      executor_, timeout)
      << "'var2' has not been published yet after " << timeout.count() << " ms";

    bookkeeping.disableAll();
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    ASSERT_EVENTUALLY_THAT(promised_publication, Eq(true), executor_, timeout)
      << "Unable to publish stats variables after " << timeout.count() << " ms";

    last_msg_.reset();

    /// After PUBLISH_ASYNC_STATISTICS() the actual publication of the statistics
    /// by StatisticsRegistry::publisherThreadCycle() is indeterministic.
    /// To prevent flakiness, we wait for a reasonable amount of time
    /// that the promised statistics gets actually published
    EXPECT_EVENTUALLY_THAT(
      get_variables,
      UnorderedElementsAre(
        "topic_stats." + statistics_topic + ".publish_async_attempts",
        "topic_stats." + statistics_topic + ".publish_async_failures",
        "topic_stats." + statistics_topic + ".publish_buffer_full_errors",
        "topic_stats." + statistics_topic + ".last_async_pub_duration"),
      executor_, timeout)
      << "'var2' has not been published yet after " << timeout.count() << " ms";
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::splitMsgTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;

  std::string topic("other_topic");
  RegistrationsRAII bookkeeping;
  REGISTER_VARIABLE(node_, statistics_topic, "macro_var1", &var1_, NULL);
  REGISTER_VARIABLE(node_, statistics_topic, "macro_var1_bk", &var1_, &bookkeeping);
  REGISTER_VARIABLE(node_, statistics_topic, "macro_var2", &var2_, NULL);
  REGISTER_VARIABLE_SIMPLE(node_, statistics_topic, &var2_, &bookkeeping);
  // Time for publisher to connect to subscriber
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  uint32_t old_names_version = 0;
  for (int i = 0; i < 4; ++i) {
    for (bool remove : {false, true}) {
      if (!remove) {
        REGISTER_VARIABLE(node_, statistics_topic, "var" + std::to_string(i), &var2_, NULL);
      } else {
        UNREGISTER_VARIABLE(node_, statistics_topic, "var" + std::to_string(i));
      }
      SCOPED_TRACE(
        std::string("Iteration ") + std::to_string(i) + " remove " +
        std::to_string(remove));

      resetMsgs();
      PUBLISH_STATISTICS(node_, statistics_topic);
      ASSERT_TRUE(waitForMsg());
      ASSERT_EQ(last_msg_->header.stamp, last_names_msg_->header.stamp);
      ASSERT_EQ(last_names_msg_->header.stamp, last_values_msg_->header.stamp);
      ASSERT_EQ(last_names_msg_->names_version, last_values_msg_->names_version);
      ASSERT_GT(last_names_msg_->names_version, old_names_version);
      old_names_version = last_names_msg_->names_version;

      ASSERT_EQ(getVariables(*last_msg_), last_names_msg_->names);

      resetMsgs();
      PUBLISH_STATISTICS(node_, statistics_topic);
      ASSERT_TRUE(waitForMsg());
      ASSERT_LT(
        rclcpp::Time(last_names_msg_->header.stamp),
        rclcpp::Time(last_values_msg_->header.stamp));
      ASSERT_EQ(last_names_msg_->names_version, last_values_msg_->names_version);
      ASSERT_EQ(last_names_msg_->names_version, old_names_version);
    }
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::callStartPublishThreadMultipleTimes()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;

  for (int ii = 0; ii < 200; ++ii) {
    START_PUBLISH_THREAD(node_, statistics_topic);
  }
}

template<typename NodeT>
void PalStatisticsTestHelperClass<NodeT>::startStopPublishThreadTest()
{
  const std::string statistics_topic = std::string(node_->get_name()) + "/" +
    DEFAULT_STATISTICS_TOPIC;
  START_PUBLISH_THREAD(node_, statistics_topic);
  REGISTER_VARIABLE(node_, statistics_topic, "macro_var1", &var1_, NULL);
  waitForMsg();

  STOP_PUBLISHER_THREAD(node_, statistics_topic);
  EXPECT_FALSE(last_msg_.get());
}

TEST_F(PalStatisticsTest, stressAsync)
{
  node_test_->stressAsyncTest();
  lifecycle_test_->stressAsyncTest();
}

TEST_F(PalStatisticsTest, concurrencyTest)
{
  node_test_->concurrencyTest();
  lifecycle_test_->concurrencyTest();
}

TEST_F(PalStatisticsTest, concurrencyMixTest)
{
  node_test_->concurrencyMixTest();
  lifecycle_test_->concurrencyMixTest();
}

TEST_F(PalStatisticsTest, singlePublish)
{
  node_test_->singlePublishTest();
  lifecycle_test_->singlePublishTest();
}

TEST_F(PalStatisticsTest, chaosTest)
{
  node_test_->chaosTest();
  lifecycle_test_->chaosTest();
}

TEST_F(PalStatisticsTest, chaosTest2)
{
  node_test_->chaosTest2();
  lifecycle_test_->chaosTest2();
}

TEST_F(PalStatisticsTest, chaosTest3)
{
  node_test_->chaosTest3();
  lifecycle_test_->chaosTest3();
}

TEST_F(PalStatisticsTest, chaosTest4)
{
  node_test_->chaosTest4();
  lifecycle_test_->chaosTest4();
}

TEST_F(PalStatisticsTest, scopedBookkeepingTest)
{
  node_test_->scopedBookkeepingTest();
  lifecycle_test_->scopedBookkeepingTest();
}

TEST_F(PalStatisticsTest, bookkeepingDisabledByDefault)
{
  node_test_->bookkeepingDisabledByDefault();
  lifecycle_test_->bookkeepingDisabledByDefault();
}

TEST_F(PalStatisticsTest, splitMsgTest)
{
  node_test_->splitMsgTest();
  lifecycle_test_->splitMsgTest();
}

TEST_F(PalStatisticsTest, callStartPublishThreadMultipleTimes)
{
  node_test_->callStartPublishThreadMultipleTimes();
  lifecycle_test_->callStartPublishThreadMultipleTimes();
}

TEST_F(PalStatisticsTest, startStopPublishThreadTest)
{
  node_test_->startStopPublishThreadTest();
  lifecycle_test_->startStopPublishThreadTest();
}
