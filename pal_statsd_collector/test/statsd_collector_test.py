#!/usr/bin/env python

import rospy
import unittest
from pal_statistics_msgs.msg import Statistic
from pal_statistics_msgs.msg import Statistics
from pal_statsd_collector.statsd_collector import StatsdCollector

SYSTEM_NAME = 'test_system'

class StatsdCollectorTest(unittest.TestCase):

    def setUp(self):
        self.topics = [{ 'name': 'test_topic', \
                        'stats': [ \
                                  { 'name': 'stat_1', 'type': ['c'] }, \
                                  { 'name': 'stat_2', 'type': ['g'] }, \
                                  { 'name': 'stat_3', 'type': ['t'] } \
                                  ]}]
        self.sc = StatsdCollector(topics=self.topics)

    def test_stat_match(self):
        self.assertTrue( self.sc.stat_match('stat.status', 'stat.status') )

        self.assertTrue( self.sc.stat_match('stat1.status', 'stat[1-2].status') )
        self.assertTrue( self.sc.stat_match('stat2.status', 'stat[1-2].status') )

        self.assertTrue( self.sc.stat_match('stat.pre_post', 'stat.pre_*') )
        self.assertTrue( self.sc.stat_match('stat.pre_post', 'stat.*_post') )

        self.assertTrue( self.sc.stat_match('stat.pre.status_1', 'stat.pre.*') )
        self.assertTrue( self.sc.stat_match('stat.pre.status_2', 'stat.pre.*') )
        self.assertTrue( self.sc.stat_match('stat.pre.status_3', 'stat.pre.*') )

        self.assertTrue( self.sc.stat_match('stat.status', '.*.status') )

    def test_statistics_callback(self):
        msg = Statistics()
        msg.statistics.append(Statistic('stat_1', 1.0))
        msg.statistics.append(Statistic('stat_2', 2.0))
        msg.statistics.append(Statistic('stat_3', 3.0))
        try:
            self.sc.statistics_callback(msg, self.topics[0]['stats'])
        except:
            self.fail("Exception raised!")

if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_node')
    rosunit.unitrun('pal_statsd_collector', 'statsd_collector_test', StatsdCollectorTest)

