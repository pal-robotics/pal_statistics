#!/usr/bin/env python

import graphitesend
import rospy
import sys
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


if __name__ == "__main__":
    rospy.init_node('carbon_collector')

    if not rospy.has_param('~topics'):
        print "No topics were specified"
        sys.exit(1)

    topics = rospy.get_param('~topics')

    dry_run = False
    if rospy.has_param('~dry_run'):
        dry_run = rospy.get_param('~dry_run')

    carbon_collector = CarbonCollector(topics, dry_run=dry_run)

    rospy.spin()

    sys.exit(0)
