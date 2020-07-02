#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import unittest
import rosbag
import roslaunch
import rospy
import rostest
import os
import sys
from std_msgs.msg import *

class LatchedPub(unittest.TestCase):

  def test_latched_pub(self):
    rospy.init_node('latched_pub')

    # Wait a while before publishing
    rospy.sleep(rospy.Duration.from_sec(5.0))

    pub= rospy.Publisher("chatter", String, latch=True)

    pub.publish(String("hello"))

    rospy.sleep(rospy.Duration.from_sec(5.0))
    
  def test_latched_split(self):
    pub = rospy.Publisher("chatter2", String, latch=True)

    pub.publish(String("hello"))

    # Wait for some time before starting the bag recording
    rospy.sleep(rospy.Duration.from_sec(5.0))

    # Kick off a rosbag record that splits every 2 seconds
    rosbag_args = "--split --repeat-latched --duration=2s chatter2 -O /tmp/test_latched_pub_delayed_sub"

    node = roslaunch.core.Node(package="rosbag", node_type="record", name="repeat_recorder", args=rosbag_args)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    # Give the record time to split
    rospy.sleep(rospy.Duration.from_sec(6.0))

    # Shut down the roslaunch instance to stop bagging
    launch.stop()

    # Now check that the topic is in each of the bag files
    bag_count = 0
    previous_time = rospy.Time()

    for file in os.listdir("/tmp/"):
        if file.startswith("test_latched_pub_delayed_sub") and file.endswith(".bag"):
            bag_count += 1

            bag_file_name = os.path.join("/tmp", file)
            bag_file = rosbag.Bag(bag_file_name, 'r')

            messages_in_bag = 0

            for _, msg, t in bag_file.read_messages(topics=['chatter2']):
                self.assertNotEqual(previous_time, t)
                self.assertEqual("hello", msg.data)
                messages_in_bag += 1
                previous_time = t

            self.assertEqual(1, messages_in_bag)

    self.assertGreater(bag_count, 1)

if __name__ == '__main__':
  rostest.rosrun('test_rosbag', 'latched_pub', LatchedPub, sys.argv)
