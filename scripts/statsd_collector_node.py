#!/usr/bin/env python

import rospy
import sys
from pal_statsd_collector.statsd_collector import StatsdCollector

if __name__ == "__main__":
    rospy.init_node('statsd_collector_node')

    statsd_collector = StatsdCollector()

    rospy.spin()

    sys.exit(0)
