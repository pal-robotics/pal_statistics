#!/usr/bin/env python

import pystatsd
import rospy
import socket
from pal_statistics_msgs.msg import Statistics

class StatsdCollector:
    def __init__(self, topics):
        self.fqdn = socket.getfqdn()
        self.pc = pystatsd.Client(prefix=self.fqdn)

        self.statistics_subs = []
        self.stats_cfg = {}
        for topic in topics:
            statistics_sub = rospy.Subscriber(topic['name'], Statistics, self.statistics_callback)
            self.statistics_subs.append(statistics_sub)

            for stat in topic['stats']:
                self.stats_cfg[stat['name']] = stat['type']

    def statistics_callback(self, msg):
        for stat in msg.statistics:
            stat_type = self.stats_cfg.get(stat.name)
            if stat_type != None:
                if 'c' in stat_type or 'counter' in stat_type:
                    self.pc.increment(stat.name, stat.value)
                if 'g' in stat_type or 'gauge' in stat_type:
                    self.pc.gauge(stat.name, stat.value)
                if 't' in stat_type or 'timer' in stat_type:
                    self.pc.timing(stat.name, stat.value)

