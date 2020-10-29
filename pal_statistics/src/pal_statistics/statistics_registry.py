#!/usr/bin/env python

# Copyright 2020 PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PAL Robotics S.L. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from pal_statistics_msgs.msg import Statistic, Statistics, StatisticsNames, StatisticsValues
import rospy


class Registration:
    """
    An utility class to handle to a registered function or variable.

    When goes out of scope unregisters the variable.
    """

    def __init__(self, name, registry):
        self.name = name
        self.registry = registry

    def __del__(self):
        rospy.logdebug('Unregistering ' + self.name)
        self.registry.unregister(self.name)


class StatisticsRegistry:

    def __init__(self, topic):
        self.topic = topic
        self.functions = {}
        self.full_pub = rospy.Publisher(topic + '/full', Statistics, queue_size=1)
        self.names_pub = rospy.Publisher(
            topic + '/names',
            StatisticsNames,
            queue_size=1,
            latch=True)
        self.values_pub = rospy.Publisher(topic + '/values', StatisticsValues, queue_size=1)
        self.names_changed = True
        self.last_names_version = 1

    # Python will copy the value of variables of simple types
    # such as numbers, we cannot store them as such
    # def registerVariable(self, name, variable):
        # self.variables[name] = variable

    def registerFunction(self, name, func, registration_list=None):
        """
        Register a function to retrieve the value to be published.

        func will be called to read the value to be published when the publish() method is called.

        @param registration_list: If not None, will be extended to include a
        Registration object for the registered function

        The function takes no arguments and returns a value convertable to float
        It can also be used to register a variable using lambda

        registerFunction('my_function', self.my_function)
        registerFunction('my_variable', (lambda: variable))
        """
        self.functions[name] = func
        if registration_list is not None:
            registration_list.append(Registration(name, self))
        self.names_changed = True

    def unregister(self, name):
        """Unregister a function or variable so it's no longer read."""
        try:
            self.functions.pop(name)
        except KeyError as e:
            rospy.logerr('Error unregistering ' + name + e.what())
        self.names_changed = True

    def publish(self):
        if self.full_pub.get_num_connections() > 0:
            self.full_pub.publish(self.createFullMsg())

        # When name changes, we need to publish to keep the latched topic in the latest version
        if self.names_changed or self.values_pub.get_num_connections() > 0:
            names_msg, values_msg = self.createOptimizedMsgs()
            if names_msg:
                self.names_pub.publish(names_msg)
            if values_msg:
                self.values_pub.publish(values_msg)

    def publishCustomStatistic(self, name, value):
        """Publish a one-time statistic."""
        msg = Statistics()
        msg.header.stamp = rospy.Time.now()
        s = Statistic()
        s.name = name
        s.value = value
        msg.statistics.append(s)
        self.full_pub.publish(msg)

    def publishCustomStatistics(self, msg):
        """Publish a one-time statistics msg."""
        self.full_pub.publish(msg)

    def createFullMsg(self):
        """Create and return a message after reading all registrations."""
        msg = Statistics()
        msg.header.stamp = rospy.Time.now()
        for name, func in self.functions.iteritems():
            s = Statistic()
            s.name = name
            s.value = func()
            msg.statistics.append(s)
        return msg

    def createOptimizedMsgs(self):
        """Create and return a names, values message after reading all registrations."""
        values_msg = StatisticsValues()
        values_msg.header.stamp = rospy.Time.now()
        if self.names_changed:
            names_msg = StatisticsNames()
            names_msg.header.stamp = values_msg.header.stamp
            self.last_names_version += 1
            names_msg.names_version = self.last_names_version
        else:
            names_msg = None
        values_msg.names_version = self.last_names_version

        for name, func in self.functions.iteritems():
            if names_msg:
                names_msg.names.append(name)
            values_msg.values.append(func())
        self.names_changed = False
        return names_msg, values_msg
