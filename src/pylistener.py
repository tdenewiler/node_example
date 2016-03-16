#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

# Import required Python code.
import rospy

# Import custom message data.
from node_example.msg import NodeExampleData

def callback(data):
    '''
    Callback function for the subscriber.
    '''
    # Simply print out values in our custom message.
    rospy.loginfo(rospy.get_name() + " I heard %s", data.message +
        ", a + b = %d" % (data.a + data.b))

def listener():
    '''
    Main function.
    '''
    # Create a subscriber with appropriate topic, custom message and name of
    # callback function.
    rospy.Subscriber('example', NodeExampleData, callback)
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pylistener')
    # Go to the main loop.
    listener()
