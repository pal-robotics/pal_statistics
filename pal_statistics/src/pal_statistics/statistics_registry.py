#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_statistics: pal_statistics.py
#
# Copyright (c) 2018 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Victor Lopez, Jordan Palacios


import rospy
from pal_statistics_msgs.msg import Statistics, Statistic


class Registration:
    """
    A utility class to handle to a registered funciton or variable, when
    out of scope unregisters the variable.
    """
    def __init__(self, name, registry):
        self.name = name
        self.registry = registry

    def __del__(self):
        rospy.logdebug("Unregistering " + self.name)
        self.registry.unregister(self.name)

class StatisticsRegistry:

    def __init__(self, topic):
        self.topic = topic
        self.functions = {}
        self.pub = rospy.Publisher(topic, Statistics, queue_size=1)

    # Python will copy the value of variables of simple types
    # such as numbers, we cannot store them as such
    # def registerVariable(self, name, variable):
        # self.variables[name] = variable

    def registerFunction(self, name, func, registration_list=None):
        """
        Registers a function that will be called to read the value to
        be published when the publish() method is called

        @param registration_list: If not None, will be extended to include a
        Registration object for the registered function

        The function takes no arguments and returns a value convertable to float
        It can also be used to register a variable using lambda

        registerFunction("my_function", self.my_function)
        registerFunction("my_variable", (lambda: variable))
        """
        self.functions[name] = func
        if registration_list is not None:
            registration_list .append(Registration(name, self))

    def unregister(self, name):
        """
        Unregisters a function or variable so it's no longer read
        """
        try:
            self.functions.pop(name)
        except KeyError as e:
            rospy.logerr("Error unregistering " + name + e.what())

    def publish(self):
        self.pub.publish(self.createMsg())

    def publishCustomStatistic(self, name, value):
        """
        Publishes a one-time statistic
        """
        msg = Statistics()
        msg.header.stamp = rospy.Time.now()
        s = Statistic()
        s.name = name
        s.value = value
        msg.statistics.append(s)
        self.pub.publish(msg)

    def publishCustomStatistics(self, msg):
        """
        Publishes a one-time statistics msg
        """
        self.pub.publish(msg)

    def createMsg(self):
        """
        Create and return a message after reading all registrations
        """
        msg = Statistics()
        msg.header.stamp = rospy.Time.now()
        for name, func in self.functions.iteritems():
            s = Statistic()
            s.name = name
            s.value = func()
            msg.statistics.append(s)
        return msg
