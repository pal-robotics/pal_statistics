#!/usr/bin/env python

import rospy
import unittest
from pal_carbon_collector.carbon_collector import CarbonCollector
from pal_statistics_msgs.msg import Statistic
from pal_statistics_msgs.msg import Statistics

SYSTEM_NAME = 'test_system'

class CarbonCollectorTest(unittest.TestCase):

    def setUp(self):
        self.cc = CarbonCollector(topics=[], system_name=SYSTEM_NAME, dry_run=True)

    # no statistics
    def test_statistics_1(self):
        msg = Statistics()
        result = self.cc.statistics_callback(msg)
        self.assertEquals(result, '')

    # one statistic with default stamp
    def test_statistics_2(self):
        msg = Statistics()

        msg.header.stamp = rospy.Time()

        stat = Statistic()
        stat.name = "test_stat"
        stat.value = 1234.0
        msg.statistics.append(stat)

        result = self.cc.statistics_callback(msg)
        output = result.splitlines()
        self.assertEquals(len(output), 1)
        self.assertEquals(output[0], 'test_system.test_stat 1234.000000 0')

    # statistics with default stamp
    def test_statistics_3(self):
        msg = Statistics()

        msg.header.stamp = rospy.Time()

        num_stats = 5
        for i in range(num_stats):
            stat = Statistic()
            stat.value = i
            stat.name = "test_stat_" + str(i)
            msg.statistics.append(stat)

        result = self.cc.statistics_callback(msg)
        output = result.splitlines()
        self.assertEquals(len(output), num_stats)
        for i in range(num_stats):
            parts = output[i].split(' ')
            self.assertEquals(parts[0], 'test_system.test_stat_' + str(i))
            self.assertEquals(float(parts[1]), float(i))
            self.assertEquals(parts[2], '0')

if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_node')
    rosunit.unitrun('pal_carbon_collector', 'carbon_collector_test', CarbonCollectorTest)

