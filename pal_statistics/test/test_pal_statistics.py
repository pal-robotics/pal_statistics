#!/usr/bin/env python

from builtins import range

import unittest
import rospy
import gc
from pal_statistics import StatisticsRegistry
from pal_statistics_msgs.msg  import Statistics, StatisticsNames, StatisticsValues

DEFAULT_TOPIC = "pal_statistics"
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
            self.assertListEqual(list(expected.keys()), names_msg.names)
        self.assertListEqual(list(expected.values()), values_msg.values)


    def evaluate_msgs(self, expected, registry):
        full_msg = registry.createFullMsg()
        self.compare_full_msg(expected, full_msg)
        names_msg, values_msg = registry.createOptimizedMsgs()
        self.compare_optimized_msgs(expected, names_msg, values_msg)

    def test_basic(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registry.registerFunction("var1", (lambda: var1))

        self.evaluate_msgs({"var1": 0.0}, registry)
        var1 = 1.0

        msg = registry.createFullMsg()
        self.evaluate_msgs({"var1": 1.0}, registry)
        var2 = 2

        registry.registerFunction("var2", (lambda: var2))
        msg = registry.createFullMsg()
        self.evaluate_msgs({"var1": 1.0, "var2": 2.0}, registry)

        registry.unregister("var1")
        self.evaluate_msgs({"var2": 2.0}, registry)

    def test_list_data(self):
        list_data = [0, 10, 20, 30]
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        for i in range(0, len(list_data)):
            registry.registerFunction("var_{}".format(i), (lambda index=i: list_data[index]))

        self.evaluate_msgs({"var_0": 0.0, "var_1": 10.0, "var_2": 20.0, "var_3": 30.0}, registry)

        # Testing by modifying the data in the list
        list_data[2] = 152.2
        list_data[3] = 1.23
        self.evaluate_msgs({"var_0": 0.0, "var_1": 10.0, "var_2": 152.2, "var_3": 1.23}, registry)

    def test_map_data(self):
        map_data = {"x": 20.0, "y": 124.2, "z": 20}
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        for key in map_data:
            registry.registerFunction("var_{}".format(key), (lambda map_key=key: map_data[map_key]))

        self.evaluate_msgs({"var_x": 20.0, "var_y": 124.2, "var_z": 20}, registry)

        # Testing by modifying the data in the dictionary
        map_data["x"] = 25.7
        map_data["z"] += 7
        self.evaluate_msgs({"var_x": 25.7, "var_y": 124.2, "var_z": 27}, registry)

    def test_registration_list(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registration_list = []
        registry.registerFunction("var1", (lambda: var1), registration_list)

        self.evaluate_msgs({"var1": 0.0}, registry)

        del registration_list
        self.evaluate_msgs({}, registry)

    def test_publish(self):
        sub = rospy.Subscriber(DEFAULT_TOPIC + "/full", Statistics,
                               self.full_msg_cb)
        names_sub = rospy.Subscriber(DEFAULT_TOPIC + "/names", StatisticsNames,
                               self.names_msg_cb)
        values_sub = rospy.Subscriber(DEFAULT_TOPIC + "/values", StatisticsValues,
                               self.values_msg_cb)
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        rospy.sleep(0.5) #Time for pub-sub connection
        self.clear()
        registry.publish()
        self.wait_for_msg()
        self.compare_full_msg({}, self.last_full_msg)
        self.assertEqual(self.last_names_msg.names_version,
                         self.last_values_msg.names_version)
        old_names_ver = self.last_names_msg.names_version

        var = 1.0
        registry.registerFunction("var", (lambda: var))
        self.clear()
        registry.publish()
        self.wait_for_msg()
        self.compare_full_msg({"var": 1.0}, self.last_full_msg)
        self.assertEqual(self.last_names_msg.names_version,
                         self.last_values_msg.names_version)
        self.assertNotEqual(old_names_ver, self.last_names_msg.names_version)
        old_names_ver = self.last_names_msg.names_version

        #If we publish the same statistics, names_version shouldn't change
        self.clear(False)
        registry.publish()
        self.wait_for_msg()
        self.assertEqual(old_names_ver, self.last_names_msg.names_version)


        self.clear()
        registry.publishCustomStatistic("foo", 23)
        rospy.sleep(0.2) #hard coded sleep only for full msg
        self.compare_full_msg({"foo": 23}, self.last_full_msg)


    def clear(self, expect_new_names=True):
        self.last_full_msg = None
        self.last_values_msg = None
        if expect_new_names:
            self.last_names_msg = None

    def wait_for_msg(self, timeout=2.0):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            if self.last_full_msg is not None and self.last_values_msg is not None and self.last_names_msg is not None:
                return
            rospy.sleep(0.1)
        raise rospy.ROSException("Timeout waiting for msg")



if __name__ == '__main__':
    import rostest
    rospy.init_node("pal_statistics_test", log_level=rospy.INFO)
    rostest.rosrun('pal_statistics', 'test_pal_statistics', TestPalStatistics)
