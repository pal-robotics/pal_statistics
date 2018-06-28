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
        for topic in topics:
            statistics_sub = rospy.Subscriber(topic, Statistics, self.statistics_callback)
            self.statistics_subs.append(statistics_sub)

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

