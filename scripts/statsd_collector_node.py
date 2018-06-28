#!/usr/bin/env python

import pystatsd
import rospy
import socket
import sys
from pal_statistics_msgs.msg import Statistics


class StatsdCollector:
    def __init__(self):
        self.fqdn = socket.getfqdn()
        self.pc = pystatsd.Client(prefix=self.fqdn)

        self.statistics_sub = rospy.Subscriber('/pal_statistics', Statistics, self.statistics_callback)

        self.stats_cfg = { 'pb18.temperature' : ['t'], 'pb17.temperature' : ['t'], 'pb17.charge' : ['g'] }

    def statistics_callback(self, msg):
        for stat in msg.statistics:
            stat_type = self.stats_cfg.get(stat.name)
            if stat_type != None:
                if 'c' in stat_type:
                    self.pc.increment(stat.name, stat.value)
                if 'g' in stat_type:
                    self.pc.gauge(stat.name, stat.value)
                if 't' in stat_type:
                    self.pc.timing(stat.name, stat.value)


if __name__ == "__main__":
    rospy.init_node('statsd_collector_node')

    statsd_collector = StatsdCollector()

    rospy.spin()

    sys.exit(0)
