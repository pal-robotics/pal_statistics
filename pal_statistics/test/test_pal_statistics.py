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

from pal_statistics.statistics_registry import StatisticsRegistry
from pal_statistics_msgs.msg import Statistics, StatisticsNames, StatisticsValues
import rclpy
from rclpy.duration import Duration
import rclpy.exceptions
from rclpy.logging import LoggingSeverity, set_logger_level
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


DEFAULT_TOPIC = 'pal_statistics'


class TestPalStatistics(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()

        self.node = Node('pal_statistics_test')
        self.logger = self.node.get_logger()
        set_logger_level('test_module', LoggingSeverity.INFO)

    @classmethod
    def tearDownClass(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.sub = None
        self.names_sub = None
        self.values_sub = None

    def tearDown(self):
        if self.sub:
            self.sub.destroy()
        if self.names_sub:
            self.names_sub.destroy()
        if self.values_sub:
            self.values_sub.destroy()
        pass

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
            self.assertListEqual(list(expected.keys()), names_msg.names)
        self.assertListEqual(list(expected.values()), list(values_msg.values))

    def evaluate_msgs(self, expected, registry):
        full_msg = registry.createFullMsg()
        self.compare_full_msg(expected, full_msg)
        names_msg, values_msg = registry.createOptimizedMsgs()
        self.compare_optimized_msgs(expected, names_msg, values_msg)

    def test_basic(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC, self.node)
        registry.registerFunction('var1', (lambda: var1))

        self.evaluate_msgs({'var1': 0.0}, registry)
        var1 = 1.0

        self.evaluate_msgs({'var1': 1.0}, registry)
        var2 = 2.0

        registry.registerFunction('var2', (lambda: var2))
        self.evaluate_msgs({'var1': 1.0, 'var2': 2.0}, registry)

        registry.unregister('var1')
        self.evaluate_msgs({'var2': 2.0}, registry)

    def test_list_data(self):
        list_data = [0.0, 10.0, 20.0, 30.0]
        registry = StatisticsRegistry(DEFAULT_TOPIC, self.node)
        for i in range(0, len(list_data)):
            registry.registerFunction('var_{}'.format(i), (lambda index=i: list_data[index]))

        self.evaluate_msgs({'var_0': 0.0, 'var_1': 10.0, 'var_2': 20.0, 'var_3': 30.0}, registry)

        # Testing by modifying the data in the list
        list_data[2] = 152.2
        list_data[3] = 1.23
        self.evaluate_msgs({'var_0': 0.0, 'var_1': 10.0, 'var_2': 152.2, 'var_3': 1.23}, registry)

    def test_map_data(self):
        map_data = {'x': 20.0, 'y': 124.2, 'z': 20.0}
        registry = StatisticsRegistry(DEFAULT_TOPIC, self.node)
        for key in map_data:
            registry.registerFunction('var_{}'.format(
                key), (lambda map_key=key: map_data[map_key]))

        self.evaluate_msgs({'var_x': 20.0, 'var_y': 124.2, 'var_z': 20}, registry)

        # Testing by modifying the data in the dictionary
        map_data['x'] = 25.7
        map_data['z'] += 7
        self.evaluate_msgs({'var_x': 25.7, 'var_y': 124.2, 'var_z': 27}, registry)

    def test_registration_list(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC, self.node)
        registration_list = []
        registry.registerFunction('var1', (lambda: var1), registration_list)

        self.evaluate_msgs({'var1': 0.0}, registry)

        del registration_list
        self.evaluate_msgs({}, registry)

    def test_publish(self):
        names_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=QoSReliabilityPolicy.RELIABLE)
        data_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.sub = self.node.create_subscription(
            Statistics, DEFAULT_TOPIC + '/full', self.full_msg_cb,
            qos_profile=data_qos)
        self.names_sub = self.node.create_subscription(
            StatisticsNames, DEFAULT_TOPIC + '/names',  self.names_msg_cb,
            qos_profile=names_qos)
        self.values_sub = self.node.create_subscription(
            StatisticsValues, DEFAULT_TOPIC + '/values', self.values_msg_cb,
            qos_profile=data_qos)

        rclpy.spin_once(self.node, timeout_sec=0.5)
        registry = StatisticsRegistry(DEFAULT_TOPIC, self.node)

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
        self.clear(expect_new_names=False)
        registry.publish()
        self.wait_for_msg(5.0)
        self.assertEqual(old_names_ver, self.last_names_msg.names_version)

        self.clear()
        registry.publishCustomStatistic('foo', 23.0)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        self.compare_full_msg({'foo': 23}, self.last_full_msg)

    def clear(self, expect_new_names=True):
        self.last_full_msg = None
        self.last_values_msg = None
        if expect_new_names:
            self.last_names_msg = None

    def wait_for_msg(self, timeout=2.0):

        end = self.node.get_clock().now() + Duration(seconds=timeout)

        while self.node.get_clock().now() < end:

            if (self.last_full_msg is not None and self.last_values_msg is not None and
                    self.last_names_msg is not None):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise TimeoutError('Timeout waiting for msg')


if __name__ == '__main__':
    unittest.main()
