#!/usr/bin/env python

import rospy
import sys
from pal_statsd_collector.statsd_collector import StatsdCollector

if __name__ == "__main__":
    rospy.init_node('statsd_collector_node')

    if not rospy.has_param('~topics'):
        print "No topics were specified"
        sys.exit(1)

    topics = rospy.get_param('~topics')

    statsd_collector = StatsdCollector(topics)

    rospy.spin()

    sys.exit(0)
