^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_statistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.2 (2025-02-05)
------------------
* Reworked all pubilsher QoS policies
  Specially important was avoiding keeping all messages, this was running
  away with memory. Also, we switch to a best effort reliability instead
  of reliable, no sense in retrying to send old data.
* Contributors: Jordan Palacios

2.6.1 (2025-01-30)
------------------
* Merge branch 'add/sai/maintainer' into 'humble-devel'
  Add Sai to the package maintainers
  See merge request qa/pal_statistics!51
* Add Sai to the package maintainers
* Merge branch 'use/core/libboost' into 'humble-devel'
  Use libboost-dev instead of whole boost libraries
  See merge request qa/pal_statistics!50
* Use libboost-dev instead of whole boost libraries
* Merge branch 'fix/includes/clang' into 'humble-devel'
  use std::optional to initialize properly instead of -1
  See merge request qa/pal_statistics!49
* use std::optional to initialize properly instead of -1
* Contributors: Jordan Palacios, Sai Kishor Kothakota

2.6.0 (2024-12-05)
------------------

2.5.1 (2024-12-05)
------------------
* Merge branch 'fix/crash/on_destruction' into 'humble-devel'
  Fix pal_statistics crash upon destruction
  See merge request qa/pal_statistics!48
* Add missing setEnabledmpl in registerInternal method
* Add DELETE_REGISTRY and CLEAR_ALL_REGISTRIES macros
* Check if the registry exists before calling `publishAsync`
* Add clearAllRegistries method
* Add deleteRegistry method in the macros.hpp
* Catch exceptions thrown in the publisher thread
* Merge branch 'make/topics/latched' into 'humble-devel'
  Make all topics latched by default
  See merge request qa/pal_statistics!47
* Make all topics latched by default
* Contributors: Jordan Palacios, Sai Kishor Kothakota

2.5.0 (2024-11-22)
------------------
* add the documentation to the getUniqueRegistryKey method
* Add more changes to the renaming getOrCreateRegistry method
* Apply review suggestions
* Add new test on disabling the registered elements by default
* add enable option to the REGISTER_ENTITY.. macro
* Fix the cpp linter errors
* Unify getRegistry method for all types of instances of Node and LifeCycleNode
* deprecate getRegistry method
* Add support for REGISTRY_KEY on other macros
* Add getUniqueRegistryKey method for node and topic
* make getOrcreateRegistry more generic for nodes and classes
* Add scoped Bookkeeping test
* Add first test on the new macros and methods
* get resolve_topic_name method to get the proper namespaced/remapped key
* Add REGISTER_ENTITY and UNREGISTER_ENTITY macros
* Add INITIALIZE_REGISTRY macro
* Accept only if it is castable to double
* Rename the methods to getOrcreateRegistry
* add initializeRegistry method and some macros
* add createRegistry and getRegistry method for reutilizing the RegistryMap
* remove the node namespace argument
* add a way to handle the topic remappings
* Contributors: Jordan Palacios, Sai Kishor Kothakota

2.4.0 (2024-11-14)
------------------
* Create stop thread test
* Add stop thread function
* Linter: Import order and single quotes
* Contributors: David ter Kuile, Isaac Acevedo

2.3.1 (2024-08-29)
------------------
* Ensure the value is converted to float before creating the msg
* Contributors: David ter Kuile

2.3.0 (2024-08-29)
------------------
* Remove unused files
* update qos for ros2
* Port test
* Port statistics_registry.py
* Contributors: David ter Kuile

2.2.4 (2024-05-16)
------------------
* Use enabled\_.swap instead of std::swap.
  This allows this package to compile on modern g++,
  on Ubuntu 24.04.
* Contributors: Chris Lalancette

2.2.3 (2023-12-18)
------------------
* Merge branch 'fix/flaky_macro_asyncPublisher' into 'humble-devel'
  Fix/flaky macro async publisher
  See merge request qa/pal_statistics!36
* Improve test failure messages
* Make gtest failure messages more informative in aync tests
  For instance from:
  /home/user/exchange/roses/alum/pal_statistics_ws/src/pal_statistics/pal_statistics/test/gtest_pal_statistics.cpp:580: Failure
  Value of: waitFor( std::bind( stats_published_for, "macro_var1", "clp-failure", "macro_var1_bk", "macro_var2", "&var2\_"))
  Actual: false
  Expected: true
  to:
  /home/user/exchange/roses/alum/pal_statistics_ws/src/pal_statistics/pal_statistics/test/gtest_pal_statistics.cpp:673: Failure
  After 300 msValue of: get_variables
  Expected: has 9 elements and there exists some permutation of elements such that:
  - element #0 is equal to "macro_var1", and
  - element #1 is equal to "clp-failure", and
  - element #2 is equal to "macro_var1_bk", and
  - element #3 is equal to "macro_var2", and
  - element #4 is equal to "&var2\_", and
  - element #5 is equal to "topic_stats.pal_statistics_node_test/pal_statistics.publish_async_attempts", and
  - element #6 is equal to "topic_stats.pal_statistics_node_test/pal_statistics.publish_async_failures", and
  - element #7 is equal to "topic_stats.pal_statistics_node_test/pal_statistics.publish_buffer_full_errors", and
  - element #8 is equal to "topic_stats.pal_statistics_node_test/pal_statistics.last_async_pub_duration"
  Actual: { "topic_stats.pal_statistics_node_test/pal_statistics.publish_async_attempts", "topic_stats.pal_statistics_node_test/pal_statistics.publish_async_failures", "topic_stats.pal_statistics_node_test/al_statistics.publish_buffer_full_errors", "topic_stats.pal_statistics_node_test/pal_statistics.last_async_pub_duration", "macro_var1", "macro_var1_bk", "macro_var2", "&var2\_" }, which has 8 elements
* Create helpers for better failure messages in async tests
* Fix time units in messages
* Fix c++17 already enforced
* Make tests exit when requested
* Clean up unused functions
* macroTest: prevent some flakiness
* asyncPublisherTest: prevent some flakiness
* Contributors: Carles Lopez Parera, Jordan Palacios

2.2.2 (2023-11-14)
------------------
* Add website tag
* Contributors: Noel Jimenez

2.2.1 (2023-11-14)
------------------
* Merge branch 'fix/flaky_chaos_test' into 'humble-devel'
  Fix flakiness in chaos tests
  See merge request qa/pal_statistics!34
* Fix flakiness in chaos tests
* Contributors: Carles Lopez Parera, Jordan Palacios

2.2.0 (2023-10-19)
------------------
* Merge branch 'fix/crash_when_start_publish_called_twice_for_same_topic' into 'humble-devel'
  Fix: prevent crash when publisher thread is recreated
  See merge request qa/pal_statistics!33
* Use make_shared as per CR
* Test statistics publish thread can be called multiple times
* Fix: interrupt_thread flag could stay true forever
  This hinders the execution of the publisher thread, making
  it exit prematurely.
  For instance, in case joinPublisherThread() is called when no
  publisher_thread\_ is still ready:
  1. startPublishThread()
  1.1. joinPublisherThread()
  1.1.1. interrupt_thread\_ set to true
  1.1.2. publisher_thread\_ is null, no further actions
  1.2. new thread created for publisherThreadCycle()
  2. In publisherThreadCycle, interrupt_thread\_ is true
  2.1. thread finishes
* Fix use proper event to interrupt the publisher thread
* Fix: prevent crash when publisher thread is recreated
  publisher thread was destroyed before being joined causing
  the termination of the process
  See: https://en.cppreference.com/w/cpp/thread/thread/%7Ethread
* Contributors: Carles Lopez Parera, Jordan Palacios

2.1.5 (2023-04-14)
------------------
* miscellaneous enhancements
* refactor gtest_pal_statistics to test also lifecycle nodes
* add support for lifecycle nodes
* add namespace for StaticCircularBuffer
* Contributors: Noel Jimenez

2.1.4 (2023-03-02)
------------------
* Merge branch 'fix_warns' into 'humble-devel'
  Fix warns
  See merge request qa/pal_statistics!28
* change types for comparisons
* reorder variable initialization
* Contributors: Jordan Palacios, Noel Jimenez

2.1.3 (2022-09-07)
------------------
* Merge branch 'fix_linter' into 'humble-devel'
  fix linter
  See merge request qa/pal_statistics!27
* fix linter
* Contributors: Jordan Palacios, Noel Jimenez

2.1.2 (2022-09-05)
------------------

2.1.1 (2021-11-09)
------------------

2.1.0 (2021-10-15)
------------------
* Revert "Comment out tests that require galactic rclpcpp API"
  This reverts commit 6642f6a743e5d5be210f7e59191153746b296866.
* Fix cmake lint
* Contributors: Victor Lopez

2.0.0 (2021-10-15)
------------------
* Comment out tests that require galactic rclpcpp API
* Use ament_cmake_auto
* Update package.xml
* Add comment about ament_cmake_pal
* Change license to BSD-3 Clause
* Flake8 and pep257 compliance
* Add ament dependencies
* More formatting and header ordering
* Apply ament_link_cmake
* Cpplint compliance
* Rename headers to .h and uncrustify them
* Fix double comparisons in test
* Reorganize code to remove boost from include files
* Tests passing in ROS2
* Update license on headers
  refs `#5 <https://github.com/pal-robotics/pal_statistics/issues/5>`_
* Change License to MIT
  fixes `#5 <https://github.com/pal-robotics/pal_statistics/issues/5>`_
* Contributors: Victor Lopez

1.4.0 (2020-03-13)
------------------
* Merge branch 'use-atomic-bool' into 'erbium-devel'
  Use atomic bool, because we need atomic operation
  See merge request qa/pal_statistics!18
* Use atomic bool, because we need atomic operation
* Contributors: Victor Lopez, victor

1.3.1 (2019-08-28)
------------------
* Fix shadow variable warning
* Contributors: Victor Lopez

1.3.0 (2019-08-12)
------------------
* Merge branch 'async-optimizations' into 'erbium-devel'
  Optimize async update when everything is enabled
  See merge request qa/pal_statistics!17
* Smarter clear and resize of vectors when all enabled
* Optimize async update when everything is enabled
* Add LGPL3 version text
* Contributors: Victor Lopez

1.2.1 (2019-04-18)
------------------
* Fix stamp of full messages
* Contributors: Victor Lopez

1.2.0 (2019-04-16)
------------------
* Merge branch 'optimized-msg' into 'erbium-devel'
  Optimized msg
  See merge request qa/pal_statistics!15
* Update python api to new msgs
* Rename full statistics topic
* Add new messages
* Change internal structure from vector of pairs to pair of vectors
* Add missing add_dependencies
* Contributors: Victor Lopez

1.1.1 (2018-12-19)
------------------
* Merge branch 'correct-stamp' into 'erbium-devel'
  Correct stamp
  See merge request qa/pal_statistics!14
* Add macros with variable argument count
* Set time stamp from main thread
* Fix maintainer
* Contributors: Victor Lopez

1.1.0 (2018-10-29)
------------------
* Merge branch 'fix-test' into 'erbium-devel'
  Fix spurious test failure when buffer was filled
  See merge request qa/pal_statistics!12
* Fix spurious test failure when buffer was filled
* Contributors: Victor Lopez

1.0.8 (2018-10-25)
------------------
* Fix some issues with copyable object that shouldn't be
* Contributors: Victor Lopez

1.0.7 (2018-10-25)
------------------
* Change Sleep to WallSleep
  When sim time stops being published. The thread can get stuck and never
  end.
* Fix unitialized variable
* Contributors: Victor Lopez

1.0.6 (2018-10-24)
------------------
* Merge branch 'improve-constness' into 'erbium-devel'
  Change namespace to pal_statistics and and const to double *
  Closes #5
  See merge request qa/pal_statistics!11
* Add tests for registration modification between pubAsync and publishing
* Change namespace to pal_statistics and and const to double *
  Fixes https://gitlab/qa/pal_statistics/issues/5
* Contributors: Victor Lopez

1.0.5 (2018-10-24)
------------------
* Fix bug when changing registrations and publsihing before a pubAsync
* Contributors: Victor Lopez

1.0.4 (2018-10-23)
------------------
* Merge branch 'auto-start-thread' into 'erbium-devel'
  Auto start thread and use steady clock for time diff
  See merge request qa/pal_statistics!10
* Auto start thread and use steady clock for time diff
* Contributors: Victor Lopez

1.0.3 (2018-10-23)
------------------
* Merge branch 'add-extendable-registration' into 'erbium-devel'
  Add the option to customize registration
  See merge request qa/pal_statistics!9
* Add the option to customize registration
* Contributors: Victor Lopez

1.0.2 (2018-10-22)
------------------
* Increase sleep time to reduce cpu load
* Contributors: Victor Lopez

1.0.1 (2018-10-22)
------------------
* Merge branch 'add-enable' into 'erbium-devel'
  Add enable
  See merge request qa/pal_statistics!8
* Fix RT loss due to condition_variable, extend tests
* Reenable stressAsync test
* Fix publish() not publishing updated data
* Add buffer to last_values\_
* Improve const-correctness of methods
* Extend macroTest
* Add unregister variable macro and use constexpr
* Restructure mutex and other optimizations
* Add debug metrics
* Use boost variant in VariableHolder
* Remove nodehandle from buffer test
* Add enable/disable
* Add debuginfo of messages lost and set buffer size to 10
* Add message queue buffer
* Contributors: Victor Lopez

1.0.0 (2018-09-20)
------------------
* Merge branch 'python-api' into 'erbium-devel'
  First version of Python API
  See merge request qa/pal_statistics!6
* First version of Python API
* Contributors: Jordan Palacios, Victor Lopez

0.0.3 (2018-07-25)
------------------
* Fix copyright notice on test
* Acquire mutex when creating publisher thread
* Disable logs for RT safety
* Merge branch 'macros-in-lib' into 'erbium-devel'
  Put macro static registry on a lib
  See merge request qa/pal_statistics!5
* Add namespace to registry statistics
* Put macro static registry on a lib
* Contributors: Jordan Palacios, Victor Lopez

0.0.2 (2018-07-04)
------------------
* Merge branch 'add-single-publish' into 'erbium-devel'
  Add publishStatistic function
  See merge request qa/pal_statistics!3
* Add registerFunction and publishCustomStatistics
* Add namespace to getRegistry
* Add publishStatistic function
* Updated license
* Moved files to their own package directory
* Contributors: Jordan Palacios, Victor Lopez

0.0.1 (2018-06-21)
------------------
