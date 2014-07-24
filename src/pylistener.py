#!/usr/bin/env python

#
#  Title:        pylistener.py
#  Description:  ROS Python node example listener.
#

#
#      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
#      All rights reserved.
#
#      Redistribution and use in source and binary forms, with or without
#      modification, are permitted provided that the following conditions are
#      met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following disclaimer
#        in the documentation and/or other materials provided with the
#        distribution.
#      * Neither the name of the Stingray, iBotics nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
#
#      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Import required Python code.
import rospy

# Import custom message data.
from node_example.msg import NodeExampleData

# Create a callback function for the subscriber.
def callback(data):
    # Simply print out values in our custom message.
    rospy.loginfo(rospy.get_name() + " I heard %s", data.message)
    rospy.loginfo(rospy.get_name() + " a + b = %d", data.a + data.b)

# This ends up being the main while loop.
def listener():
    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber('example', NodeExampleData, callback)
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pylistener', anonymous = True)
    # Go to the main loop.
    listener()
