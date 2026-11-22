// Copyright 2026 b»robotized GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Measures the cost of publishAsync() in the calling (realtime) thread, with the
// registration pattern of ros2_control: many values registered enabled (handles) and some
// values registered disabled with bookkeeping (controller and hardware statistics).

#include <gtest/gtest.h>
#include <sys/resource.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "pal_statistics/pal_statistics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
thread_local bool g_count_allocations = false;
thread_local size_t g_allocations = 0;
}  // namespace

void * operator new(std::size_t size)
{
  if (g_count_allocations) {
    ++g_allocations;
  }
  if (void * p = std::malloc(size == 0 ? 1 : size)) {
    return p;
  }
  throw std::bad_alloc();
}

void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

namespace
{
using pal_statistics::RegistrationsRAII;
using pal_statistics::StatisticsRegistry;

constexpr size_t kEnabledValues = 450;   // state and command interface handles
constexpr size_t kDisabledValues = 40;   // controller statistics, enabled at activation
constexpr auto kCycle = std::chrono::milliseconds(2);

struct Sample
{
  double duration_us;
  long minor_faults;  // NOLINT(runtime/int)
  size_t allocations;
};

long thread_minor_faults()  // NOLINT(runtime/int)
{
  rusage usage{};
  getrusage(RUSAGE_THREAD, &usage);
  return usage.ru_minflt;
}

// Faults and allocations are counted for the calling thread only.
template<typename F>
Sample measure(F && f)
{
  const auto faults = thread_minor_faults();
  g_allocations = 0;
  g_count_allocations = true;
  const auto start = std::chrono::steady_clock::now();
  f();
  const auto end = std::chrono::steady_clock::now();
  g_count_allocations = false;
  return {
    std::chrono::duration<double, std::micro>(end - start).count(),
    thread_minor_faults() - faults, g_allocations};
}

void print(const char * label, const Sample & s)
{
  std::printf(
    "  %-34s %10.1f us  minflt %4ld  new %4zu\n", label, s.duration_us, s.minor_faults,
    s.allocations);
}

class FirstPublishTiming : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("first_publish_timing");
    registry_ = std::make_shared<StatisticsRegistry>(node_, "introspection_data");
    registry_->startPublishThread();
    for (size_t i = 0; i < kEnabledValues + kDisabledValues; ++i) {
      values_.push_back(std::make_unique<double>(static_cast<double>(i)));
    }
  }

  void TearDown() override
  {
    controller_registrations_.reset();
    registry_.reset();
  }

  // Registers one value like ros2_control does.
  void register_value(size_t i)
  {
    const double * v = values_[i].get();
    if (i < kEnabledValues) {
      registry_->registerFunction(
        "state_interface.joint" + std::to_string(i), [v]() {return *v;});
    } else {
      registry_->registerVariable(
        "controller.value" + std::to_string(i), v, controller_registrations_.get(), false);
    }
  }

  void publish_cycles(size_t n)
  {
    for (size_t i = 0; i < n; ++i) {
      registry_->publishAsync();
      std::this_thread::sleep_for(kCycle);
    }
  }

  size_t published_values()
  {
    return registry_->createMsg().statistics.size();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<StatisticsRegistry> registry_;
  std::unique_ptr<RegistrationsRAII> controller_registrations_ =
    std::make_unique<RegistrationsRAII>();
  std::vector<std::unique_ptr<double>> values_;
};

// S1: register all values, then publish. Matches the start of the controller manager.
TEST_F(FirstPublishTiming, S1_publish_after_registration)
{
  for (size_t i = 0; i < values_.size(); ++i) {
    register_value(i);
  }
  std::printf("S1: %zu values registered, then publishAsync()\n", values_.size());
  for (int i = 0; i < 5; ++i) {
    const auto s = measure([this] {registry_->publishAsync();});
    print(("publish " + std::to_string(i + 1)).c_str(), s);
    std::this_thread::sleep_for(kCycle);
  }
  publish_cycles(200);
  print("publish after 200 cycles", measure([this] {registry_->publishAsync();}));
}

// S2: publish after each registration. The queue is empty at the measured publish.
TEST_F(FirstPublishTiming, S2_publish_during_registration)
{
  for (size_t i = 0; i < values_.size(); ++i) {
    register_value(i);
    registry_->publishAsync();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::printf("S2: publishAsync() after each registration\n");
  for (int i = 0; i < 5; ++i) {
    const auto s = measure([this] {registry_->publishAsync();});
    print(("publish " + std::to_string(i + 1)).c_str(), s);
    std::this_thread::sleep_for(kCycle);
  }
}

// S3: controller activation and deactivation in steady state.
TEST_F(FirstPublishTiming, S3_enable_disable_in_steady_state)
{
  for (size_t i = 0; i < values_.size(); ++i) {
    register_value(i);
  }
  publish_cycles(200);
  std::printf("S3: steady state, then enableAll() / disableAll() of %zu values\n",
    kDisabledValues);
  print("steady publish", measure([this] {registry_->publishAsync();}));
  std::this_thread::sleep_for(kCycle);
  controller_registrations_->enableAll();
  print("publish after enableAll()", measure([this] {registry_->publishAsync();}));
  publish_cycles(200);
  print("steady publish (all enabled)", measure([this] {registry_->publishAsync();}));
  std::this_thread::sleep_for(kCycle);
  controller_registrations_->disableAll();
  print("publish after disableAll()", measure([this] {registry_->publishAsync();}));
  publish_cycles(200);
  print("steady publish (after disable)", measure([this] {registry_->publishAsync();}));
}

// S4: enable requests made before the first publish must not be lost.
TEST_F(FirstPublishTiming, S4_enable_before_first_publish)
{
  for (size_t i = 0; i < values_.size(); ++i) {
    register_value(i);
  }
  EXPECT_TRUE(controller_registrations_->enableAll()) << "enable requests did not fit the queue";
  // 4 internal topic statistics of the registry are registered and enabled.
  EXPECT_EQ(published_values(), values_.size() + 4);
}

// S5: values registered disabled are not published.
TEST_F(FirstPublishTiming, S5_disabled_values_not_published)
{
  for (size_t i = 0; i < values_.size(); ++i) {
    register_value(i);
  }
  EXPECT_EQ(published_values(), kEnabledValues + 4);
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
