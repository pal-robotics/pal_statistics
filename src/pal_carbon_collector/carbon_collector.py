#!/usr/bin/env python

import graphitesend
import rospy
from pal_statistics_msgs.msg import Statistics

class CarbonCollector:
    def __init__(self, topics, dry_run):
        self.dry_run = dry_run
        self.gs = graphitesend.GraphiteClient(prefix='', graphite_server='localhost', dryrun=dry_run)

        self.statistics_subs = []
        for topic in topics:
            statistics_sub = rospy.Subscriber(topic, Statistics, self.statistics_callback)
            self.statistics_subs.append(statistics_sub)

    def statistics_callback(self, msg):
        timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

        stats = []
        for stat in msg.statistics:
            stats.append((stat.name, stat.value))

        result = self.gs.send_list(stats, timestamp.to_sec())

        if self.dry_run:
            print result

