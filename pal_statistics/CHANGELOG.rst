^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_statistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.1 (2023-07-19)
------------------
* drop c++11 compiler flag
  broke on newer systems as log4cxx requires c++17 for quite some time now.
* Contributors: v4hn

1.5.0 (2021-08-23)
------------------
* Merge branch 'python3' into 'erbium-devel'
  Make python code compatible with python2/3
  See merge request qa/pal_statistics!23
* futurize'd python code
* Contributors: Jordan Palacios

1.4.3 (2021-03-19)
------------------
* Fix spurious test failure
* Contributors: Victor Lopez

1.4.2 (2021-01-12)
------------------
* Merge branch 'data_signal' into 'erbium-devel'
  Data signal
  See merge request qa/pal_statistics!22
* Add documentation + tests for extact_rosbag_signals
* Merge branch 'python_more_tests' into 'erbium-devel'
  add more tests based on lists and dictionary data for the python API
  See merge request qa/pal_statistics!21
* Get data signal from introspection data rosbag
  Rm *.user
  Get data signal from introspection data rosbag
  Rm *.user
  Mv DataSignal to a separate library
* add more tests based on lists and dictionary data for the python API
* Contributors: Adria Roig, Sai Kishor Kothakota, victor

1.4.1 (2020-11-11)
------------------
* Update license on headers
  refs #5
* Change License to MIT
  fixes #5
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
