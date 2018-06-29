#!/usr/bin/env python

import pystatsd
import re
import rospy
import socket
from pal_statistics_msgs.msg import Statistics

class StatsdCollector:
    def __init__(self, topics):
        self.fqdn = socket.getfqdn()
        self.pc = pystatsd.Client(prefix=self.fqdn)

        self.statistics_subs = []
        for topic in topics:
            statistics_sub = rospy.Subscriber(topic['name'], Statistics, self.statistics_callback, \
                                              callback_args=topic['stats'])
            self.statistics_subs.append(statistics_sub)

    def stat_match(self, stat_name, stat_regex):
        obj = re.match(stat_regex, stat_name)
        return obj

    def statistics_callback(self, msg, stats_rules):
        for stat in msg.statistics:
            for rule in stats_rules:
                if self.stat_match(stat.name, rule['name']):
                    stat_type = rule['type']
                    if 'c' in stat_type or 'counter' in stat_type:
                        self.pc.increment(stat.name, stat.value)
                    if 'g' in stat_type or 'gauge' in stat_type:
                        self.pc.gauge(stat.name, stat.value)
                    if 't' in stat_type or 'timer' in stat_type:
                        self.pc.timing(stat.name, stat.value)

                    # skip rest of rules after a valid match
                    continue

