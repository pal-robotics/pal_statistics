#!/usr/bin/env python

import unittest
import rospy
import gc
from pal_statistics import StatisticsRegistry
from pal_statistics_msgs.msg  import Statistics

DEFAULT_TOPIC = "pal_statistics"
class TestPalStatistics(unittest.TestCase):

    def __init__(self, *args):
        super(TestPalStatistics, self).__init__(*args)

    def msg_cb(self, msg):
        self.last_msg = msg

    def evaluate_msgs(self, expected, msg):
        actual = {}
        for i in range(0, len(msg.statistics)):
            actual[msg.statistics[i].name] = msg.statistics[i].value

        self.assertDictEqual(expected, actual)


    def test_basic(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registry.registerFunction("var1", (lambda: var1))

        self.evaluate_msgs({"var1": 0.0}, registry.createMsg())
        var1 = 1.0

        msg = registry.createMsg()
        self.evaluate_msgs({"var1": 1.0}, registry.createMsg())
        var2 = 2

        registry.registerFunction("var2", (lambda: var2))
        msg = registry.createMsg()
        self.evaluate_msgs({"var1": 1.0, "var2": 2.0}, registry.createMsg())

        registry.unregister("var1")
        self.evaluate_msgs({"var2": 2.0}, registry.createMsg())


    def test_registration_list(self):
        var1 = 0.0
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        registration_list = []
        registry.registerFunction("var1", (lambda: var1), registration_list)

        self.evaluate_msgs({"var1": 0.0}, registry.createMsg())

        del registration_list
        self.evaluate_msgs({}, registry.createMsg())

    def test_publish(self):
        sub = rospy.Subscriber(DEFAULT_TOPIC, Statistics,
                               self.msg_cb)
        registry = StatisticsRegistry(DEFAULT_TOPIC)
        self.last_msg = None
        rospy.sleep(0.5) #Time for pub-sub connection
        registry.publish()
        self.wait_for_msg()
        self.evaluate_msgs({}, self.last_msg)
        var = 1.0
        registry.registerFunction("var", (lambda: var))
        self.last_msg = None
        registry.publish()
        self.wait_for_msg()
        self.evaluate_msgs({"var": 1.0}, self.last_msg)

        self.last_msg = None
        registry.publishCustomStatistic("foo", 23)
        self.wait_for_msg()
        self.evaluate_msgs({"foo": 23}, self.last_msg)



    def wait_for_msg(self, timeout=2.0):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            if self.last_msg is not None:
                return
            rospy.sleep(0.1)
        raise rospy.ROSException("Timeout waiting for msg")



if __name__ == '__main__':
    import rostest
    rospy.init_node("pal_statistics_test", log_level=rospy.INFO)
    rostest.rosrun('pal_statistics', 'test_pal_statistics', TestPalStatistics)
