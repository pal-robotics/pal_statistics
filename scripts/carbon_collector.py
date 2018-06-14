#!/usr/bin/env python

import rospy
import graphitesend
from pal_statistics_msgs.msg import Statistics

STATISTICS_TOPIC = "pal_statistics"

class CarbonCollector:
    def __init__(self):
        self.gs = graphitesend.GraphiteClient(prefix='', graphite_server='localhost')

        self.statistics_sub = rospy.Subscriber(STATISTICS_TOPIC, Statistics, \
                                               self.statistics_callback)

    def statistics_callback(self, msg):
        timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

        stats = []
        for stat in msg.statistics:
            stats.append((stat.name, stat.value))

        self.gs.send_list(stats, timestamp.to_sec())


if __name__ == "__main__":
    rospy.init_node("carbon_collector")

    carbon_collector = CarbonCollector()

    rospy.spin()
