#!/usr/bin/env python

#
#  Title:        pytalker.py
#  Description:  ROS Python node example talker.
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
import sys

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from node_example.msg import NodeExampleData
from node_example.cfg import nodeExampleConfig as ConfigType

# Node example class.
class NodeExample():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        init_message = rospy.get_param('~message', 'hello')
        rate = float(rospy.get_param('~rate', '1.0'))
        rospy.loginfo('rate = %d', rate)
        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        # Create a publisher for our custom message.
        pub = rospy.Publisher('example', NodeExampleData)
        # Set the message to publish as our custom message.
        msg = NodeExampleData()
        # Initialize message variables.
        msg.a = 1
        msg.b = 2
        msg.message = init_message
        # Main while loop.
        while not rospy.is_shutdown():
            # Fill in custom message variables with values from dynamic reconfigure server.
            msg.message = self.message
            msg.a = self.a
            msg.b = self.b
            # Publish our custom message.
            pub.publish(msg)
            # Sleep for a while before publishing new messages. Division is so rate != period.
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.message = config["message"]
        self.a = config["a"]
        self.b = config["b"]
        # Return the new variables.
        return config

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pytalker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = NodeExample()
    except rospy.ROSInterruptException: pass
