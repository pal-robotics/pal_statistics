#!/usr/bin/env python

import graphitesend
import rospy
import sys
from pal_statistics_msgs.msg import Statistics
from pal_carbon_collector.carbon_collector import CarbonCollector

if __name__ == "__main__":
    rospy.init_node('carbon_collector_node')

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
