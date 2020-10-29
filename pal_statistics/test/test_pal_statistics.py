#!/usr/bin/env python

# Copyright 2020 PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PAL Robotics S.L. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import unittest

from pal_statistics import StatisticsRegistry
from pal_statistics_msgs.msg import Statistics, StatisticsNames, StatisticsValues
import rospy


DEFAULT_TOPIC = 'pal_statistics'


class TestPalStatistics(unittest.TestCase):

    def __init__(self, *args):
        super(TestPalStatistics, self).__init__(*args)
        self.clear()
        self.last_names_msg = None

    def full_msg_cb(self, msg):
        self.last_full_msg = msg

    def names_msg_cb(self, msg):
        self.last_names_msg = msg

    def values_msg_cb(self, msg):
        self.last_values_msg = msg

    def compare_full_msg(self, expected, full_msg):
        actual = {}
        for i in range(0, len(full_msg.statistics)):
            actual[full_msg.statistics[i].name] = full_msg.statistics[i].value

        self.assertDictEqual(expected, actual)

    def compare_optimized_msgs(self, expected, names_msg, values_msg):
        if names_msg:
            self.assertListEqual(expected.keys(), names_msg.names)
        self.assertListEqual(expected.values(), values_msg.values)

    def evaluate_msgs(self, expected, registry):
        full_msg = registry.createFullMsg()
        self.compare_full_msg(expected, full_msg)
        names_msg, values_msg = registry.createOptimizedMsgs()
        self.compare_optimized_msgs(expected, names_msg, values_msg)

    def test_basic(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registry.registerFunction('var1', (lambda: var1))

        self.evaluate_msgs({'var1': 0.0}, registry)
        var1 = 1.0

        self.evaluate_msgs({'var1': 1.0}, registry)
        var2 = 2

        registry.registerFunction('var2', (lambda: var2))
        self.evaluate_msgs({'var1': 1.0, 'var2': 2.0}, registry)

        registry.unregister('var1')
        self.evaluate_msgs({'var2': 2.0}, registry)

    def test_registration_list(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registration_list = []
        registry.registerFunction('var1', (lambda: var1), registration_list)

        self.evaluate_msgs({'var1': 0.0}, registry)

        del registration_list
        self.evaluate_msgs({}, registry)

    def test_publish(self):
        sub = rospy.Subscriber(DEFAULT_TOPIC + '/full', Statistics, self.full_msg_cb)  # noqa: F841
        names_sub = rospy.Subscriber(DEFAULT_TOPIC + '/names', StatisticsNames,  # noqa: F841
                                     self.names_msg_cb)
        values_sub = rospy.Subscriber(DEFAULT_TOPIC + '/values', StatisticsValues,  # noqa: F841
                                      self.values_msg_cb)
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        rospy.sleep(0.5)  # Time for pub-sub connection
        self.clear()
        registry.publish()
        self.wait_for_msg()
        self.compare_full_msg({}, self.last_full_msg)
        self.assertEqual(self.last_names_msg.names_version,
                         self.last_values_msg.names_version)
        old_names_ver = self.last_names_msg.names_version

        var = 1.0
        registry.registerFunction('var', (lambda: var))
        self.clear()
        registry.publish()
        self.wait_for_msg()
        self.compare_full_msg({'var': 1.0}, self.last_full_msg)
        self.assertEqual(self.last_names_msg.names_version,
                         self.last_values_msg.names_version)
        self.assertNotEqual(old_names_ver, self.last_names_msg.names_version)
        old_names_ver = self.last_names_msg.names_version

        # If we publish the same statistics, names_version shouldn't change
        self.clear()
        registry.publish()
        self.wait_for_msg()
        self.assertEqual(old_names_ver, self.last_names_msg.names_version)

        self.clear()
        registry.publishCustomStatistic('foo', 23)
        rospy.sleep(0.2)  # hard coded sleep only for full msg
        self.compare_full_msg({'foo': 23}, self.last_full_msg)

    def clear(self):
        self.last_full_msg = None
        self.last_values_msg = None

    def wait_for_msg(self, timeout=2.0):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            if (self.last_full_msg is not None and self.last_values_msg is not None and
                    self.last_names_msg is not None):
                return
            rospy.sleep(0.1)
        raise rospy.ROSException('Timeout waiting for msg')


if __name__ == '__main__':
    import rostest
    rospy.init_node('pal_statistics_test', log_level=rospy.INFO)
    rostest.rosrun('pal_statistics', 'test_pal_statistics', TestPalStatistics)
