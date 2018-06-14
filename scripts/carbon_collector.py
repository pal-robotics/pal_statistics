#!/usr/bin/env python

import rospy
import graphitesend
from pal_statistics_msgs.msg import Statistics

class CarbonCollector:
    def __init__(self, topics):
        self.gs = graphitesend.GraphiteClient(prefix='', graphite_server='localhost')

        self.statistics_subs = []
        for topic in topics:
            statistics_sub = rospy.Subscriber(topic, Statistics, self.statistics_callback)
            self.statistics_subs.append(statistics_sub)

    def statistics_callback(self, msg):
        timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

        stats = []
        for stat in msg.statistics:
            stats.append((stat.name, stat.value))

        self.gs.send_list(stats, timestamp.to_sec())


if __name__ == "__main__":
    rospy.init_node("carbon_collector")

    topics = rospy.get_param('~topics')

    carbon_collector = CarbonCollector(topics)

    rospy.spin()
